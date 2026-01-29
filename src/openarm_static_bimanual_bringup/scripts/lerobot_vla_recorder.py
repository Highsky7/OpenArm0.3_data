#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
LeRobot VLA Data Recorder for OpenArm Bimanual Robot

Records bimanual robot joint states and camera images in LeRobot v3.0 format
for VLA (Vision-Language-Action) model training.

Features:
- 16-DOF recording: left_rev1~8 + right_rev1~8 (including grippers)
- 3 camera streams: top, wrist_left, wrist_right (256x256)
- Language instruction for VLA training
- Episode-based recording with keyboard controls
- LeRobot compatible parquet + video output

Usage:
    ros2 run openarm_static_bimanual_bringup lerobot_vla_recorder.py

Keyboard Controls:
    'r' - Start new episode recording
    's' - Stop and save current episode
    'q' - Finalize dataset and quit
"""
import json
import os
import select
import sys
import termios
import time
import tty
from datetime import datetime
from pathlib import Path
from threading import Lock
from typing import Optional

import cv2
import numpy as np
import pyarrow as pa
import pyarrow.parquet as pq
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState


class LeRobotVLARecorder(Node):
    """
    LeRobot VLA Data Recorder
    
    Records joint states and camera images at a fixed frequency and saves them
    in LeRobot v3.0 compatible format with parquet + video files.
    """
    
    # Joint ordering for dataset (16 DOF total)
    JOINT_NAMES = [
        'left_rev1', 'left_rev2', 'left_rev3', 'left_rev4',
        'left_rev5', 'left_rev6', 'left_rev7', 'left_rev8',
        'right_rev1', 'right_rev2', 'right_rev3', 'right_rev4',
        'right_rev5', 'right_rev6', 'right_rev7', 'right_rev8',
    ]
    NUM_JOINTS = 16
    
    # Camera configuration
    CAMERA_TOPICS = {
        'observation.images.top': 'camera1/cam1/color/image_raw',
        'observation.images.wrist_left': 'camera1/cam2/color/image_raw',
        'observation.images.wrist_right': 'camera1/cam3/color/image_raw',
    }
    IMAGE_SIZE = (256, 256)  # Resize target
    
    def __init__(self):
        super().__init__('lerobot_vla_recorder')
        
        # Parameters
        self.declare_parameter('record_rate', 50.0)  # Hz
        self.declare_parameter('dataset_name', 'openarm_bimanual')
        self.declare_parameter('save_dir', '~/lerobot_datasets')
        self.declare_parameter('robot_type', 'openarm_static_bimanual')
        self.declare_parameter('task_description', 'bimanual manipulation task')
        self.declare_parameter('enable_cameras', True)
        
        self.record_rate = self.get_parameter('record_rate').value
        self.dataset_name = self.get_parameter('dataset_name').value
        self.save_dir = os.path.expanduser(self.get_parameter('save_dir').value)
        self.robot_type = self.get_parameter('robot_type').value
        self.task_description = self.get_parameter('task_description').value
        self.enable_cameras = self.get_parameter('enable_cameras').value
        
        # Dataset path
        self.dataset_path = Path(self.save_dir) / self.dataset_name
        self.data_dir = self.dataset_path / 'data'
        self.videos_dir = self.dataset_path / 'videos'
        
        # State
        self.is_recording = False
        self.episode_frames: list[dict] = []
        self.episode_index = 0
        self.start_time: Optional[float] = None
        self.last_joint_state: Optional[JointState] = None
        
        # Camera state
        self.cv_bridge = CvBridge()
        self.latest_images: dict[str, Optional[np.ndarray]] = {
            key: None for key in self.CAMERA_TOPICS.keys()
        }
        self.image_lock = Lock()
        
        # Episode image buffers (for video encoding)
        self.episode_images: dict[str, list[np.ndarray]] = {
            key: [] for key in self.CAMERA_TOPICS.keys()
        }
        
        # All episodes data for final parquet
        self.all_frames: list[dict] = []
        
        # Subscriber: Joint states
        self.js_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10)
        
        # Subscribers: Cameras (if enabled)
        if self.enable_cameras:
            for key, topic in self.CAMERA_TOPICS.items():
                self.create_subscription(
                    Image
                    ,
                    topic,
                    lambda msg, k=key: self.image_callback(msg, k),
                    10
                )
                self.get_logger().info(f'Subscribed: {topic} -> {key}')
        
        # Recording timer
        self.record_timer = self.create_timer(
            1.0 / self.record_rate, self.record_callback)
        
        self._print_banner()
    
    def _print_banner(self):
        """Print startup information."""
        self.get_logger().info('=' * 55)
        self.get_logger().info('  LeRobot VLA Recorder for OpenArm Bimanual')
        self.get_logger().info('=' * 55)
        self.get_logger().info(f'  Record rate: {self.record_rate} Hz')
        self.get_logger().info(f'  Dataset: {self.dataset_path}')
        self.get_logger().info(f'  Joints: {self.NUM_JOINTS} DOF')
        self.get_logger().info(f'  Cameras: {"Enabled" if self.enable_cameras else "Disabled"}')
        self.get_logger().info(f'  Image size: {self.IMAGE_SIZE[0]}x{self.IMAGE_SIZE[1]}')
        self.get_logger().info(f'  Task: {self.task_description}')
        self.get_logger().info('')
        self.get_logger().info('  Keyboard Controls:')
        self.get_logger().info("    'r' - Start new episode")
        self.get_logger().info("    's' - Stop and save episode")
        self.get_logger().info("    'q' - Finalize and quit")
        self.get_logger().info('=' * 55)
        self.get_logger().info('  Waiting for /joint_states...')
    
    def joint_state_callback(self, msg: JointState):
        """Store latest joint state."""
        self.last_joint_state = msg
    
    def image_callback(self, msg: Image, image_key: str):
        """Store and resize latest camera image."""
        try:
            # Convert ROS Image to OpenCV
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, 'rgb8')
            
            # Resize to target size (256x256)
            resized = cv2.resize(cv_image, self.IMAGE_SIZE, interpolation=cv2.INTER_AREA)
            
            with self.image_lock:
                self.latest_images[image_key] = resized
        except Exception as e:
            self.get_logger().warn(f'Image conversion error ({image_key}): {e}')
    
    def _get_joint_positions(self) -> Optional[np.ndarray]:
        """Extract joint positions in correct order."""
        if self.last_joint_state is None:
            return None
        
        # Build position dict from joint_states
        pos_dict = {}
        for i, name in enumerate(self.last_joint_state.name):
            if i < len(self.last_joint_state.position):
                pos_dict[name] = self.last_joint_state.position[i]
        
        # Extract in correct order
        positions = []
        for joint_name in self.JOINT_NAMES:
            if joint_name in pos_dict:
                positions.append(pos_dict[joint_name])
            else:
                positions.append(0.0)  # Default if missing
        
        return np.array(positions, dtype=np.float32)
    
    def _get_current_images(self) -> dict[str, Optional[np.ndarray]]:
        """Get current images with thread safety."""
        with self.image_lock:
            return {k: v.copy() if v is not None else None 
                    for k, v in self.latest_images.items()}
    
    def record_callback(self):
        """Record current frame if recording is active."""
        if not self.is_recording or self.last_joint_state is None:
            return
        
        positions = self._get_joint_positions()
        if positions is None:
            return
        
        # Check camera availability if enabled
        if self.enable_cameras:
            current_images = self._get_current_images()
            if any(img is None for img in current_images.values()):
                # Skip frame if any camera is missing
                return
        else:
            current_images = {}
        
        timestamp = time.time() - self.start_time
        frame_index = len(self.episode_frames)
        
        frame = {
            'timestamp': timestamp,
            'frame_index': frame_index,
            'episode_index': self.episode_index,
            'observation.state': positions.copy(),
            'language_instruction': self.task_description,
            '_positions': positions.copy(),  # Temporary for action computation
        }
        
        self.episode_frames.append(frame)
        
        # Store images for video encoding (if enabled)
        if self.enable_cameras:
            for key, img in current_images.items():
                self.episode_images[key].append(img)
    
    def start_recording(self):
        """Start new episode recording."""
        if self.last_joint_state is None:
            self.get_logger().warn('Cannot start: No joint_states received yet')
            return
        
        if self.enable_cameras:
            current_images = self._get_current_images()
            missing = [k for k, v in current_images.items() if v is None]
            if missing:
                self.get_logger().warn(f'Cannot start: Missing cameras: {missing}')
                return
        
        self.episode_frames = []
        self.episode_images = {key: [] for key in self.CAMERA_TOPICS.keys()}
        self.start_time = time.time()
        self.is_recording = True
        
        self.get_logger().info(f'🔴 Episode {self.episode_index} recording started!')
    
    def stop_and_save_episode(self):
        """Stop recording and save current episode."""
        self.is_recording = False
        
        if len(self.episode_frames) < 2:
            self.get_logger().warn('Episode too short (< 2 frames), discarding')
            self.episode_frames = []
            self.episode_images = {key: [] for key in self.CAMERA_TOPICS.keys()}
            return
        
        # Compute actions (next frame's state as action)
        for i in range(len(self.episode_frames) - 1):
            next_positions = self.episode_frames[i + 1]['_positions']
            self.episode_frames[i]['action'] = next_positions
        
        # Last frame uses its own position as action
        self.episode_frames[-1]['action'] = self.episode_frames[-1]['_positions']
        
        # Remove temporary field and add to all_frames
        for frame in self.episode_frames:
            del frame['_positions']
            self.all_frames.append(frame)
        
        # Save episode videos (if cameras enabled)
        if self.enable_cameras:
            self._save_episode_videos()
        
        duration = self.episode_frames[-1]['timestamp']
        num_frames = len(self.episode_frames)
        
        self.get_logger().info(
            f'💾 Episode {self.episode_index} saved: {num_frames} frames, {duration:.1f}s')
        
        self.episode_index += 1
        self.episode_frames = []
        self.episode_images = {key: [] for key in self.CAMERA_TOPICS.keys()}
    
    def _save_episode_videos(self):
        """Save episode images as MP4 videos."""
        for image_key, images in self.episode_images.items():
            if not images:
                continue
            
            # Create video directory
            video_dir = self.videos_dir / image_key
            video_dir.mkdir(parents=True, exist_ok=True)
            
            # Video path
            video_path = video_dir / f'episode_{self.episode_index:06d}.mp4'
            
            # Write video using OpenCV
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            fps = int(self.record_rate)
            h, w = self.IMAGE_SIZE[1], self.IMAGE_SIZE[0]
            
            writer = cv2.VideoWriter(str(video_path), fourcc, fps, (w, h))
            
            for img in images:
                # Convert RGB to BGR for OpenCV
                bgr_img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
                writer.write(bgr_img)
            
            writer.release()
            
            self.get_logger().debug(
                f'  Video saved: {video_path} ({len(images)} frames)')
    
    def finalize_dataset(self):
        """Finalize and write parquet dataset."""
        if self.is_recording:
            self.stop_and_save_episode()
        
        if not self.all_frames:
            self.get_logger().warn('No data to save')
            return
        
        # Create directory structure
        self.data_dir.mkdir(parents=True, exist_ok=True)
        
        # Prepare data for parquet
        timestamps = [f['timestamp'] for f in self.all_frames]
        frame_indices = [f['frame_index'] for f in self.all_frames]
        episode_indices = [f['episode_index'] for f in self.all_frames]
        observations = np.array([f['observation.state'] for f in self.all_frames])
        actions = np.array([f['action'] for f in self.all_frames])
        language_instructions = [f['language_instruction'] for f in self.all_frames]
        
        # Create table with LeRobot v3.0 compatible schema
        table = pa.table({
            'timestamp': pa.array(timestamps, type=pa.float64()),
            'frame_index': pa.array(frame_indices, type=pa.int64()),
            'episode_index': pa.array(episode_indices, type=pa.int64()),
            'observation.state': [obs.tolist() for obs in observations],
            'action': [act.tolist() for act in actions],
            'language_instruction': pa.array(language_instructions, type=pa.string()),
        })
        
        # Write parquet file
        parquet_path = self.data_dir / 'train-00000.parquet'
        pq.write_table(table, parquet_path)
        
        # Write metadata (info.json)
        self._write_info_json()
        
        self.get_logger().info('=' * 55)
        self.get_logger().info('✅ Dataset finalized!')
        self.get_logger().info(f'   Path: {self.dataset_path}')
        self.get_logger().info(f'   Episodes: {self.episode_index}')
        self.get_logger().info(f'   Total frames: {len(self.all_frames)}')
        if self.enable_cameras:
            self.get_logger().info(f'   Videos: {self.videos_dir}')
        self.get_logger().info('=' * 55)
    
    def _write_info_json(self):
        """Write LeRobot compatible info.json metadata."""
        features = {
            'observation.state': {
                'dtype': 'float32',
                'shape': [self.NUM_JOINTS],
                'names': self.JOINT_NAMES,
            },
            'action': {
                'dtype': 'float32',
                'shape': [self.NUM_JOINTS],
                'names': self.JOINT_NAMES,
            },
            'language_instruction': {
                'dtype': 'string',
                'shape': [1],
            },
        }
        
        # Add camera features if enabled
        if self.enable_cameras:
            for key in self.CAMERA_TOPICS.keys():
                features[key] = {
                    'dtype': 'video',
                    'shape': [self.IMAGE_SIZE[1], self.IMAGE_SIZE[0], 3],  # H, W, C
                    'video_info': {
                        'video.fps': int(self.record_rate),
                        'video.codec': 'mp4v',
                    },
                }
        
        info = {
            'repo_id': f'local/{self.dataset_name}',
            'robot_type': self.robot_type,
            'fps': int(self.record_rate),
            'num_episodes': self.episode_index,
            'num_frames': len(self.all_frames),
            'features': features,
            'created_at': datetime.now().isoformat(),
        }
        
        info_path = self.dataset_path / 'info.json'
        with open(info_path, 'w') as f:
            json.dump(info, f, indent=2)


def main(args=None):
    rclpy.init(args=args)
    node = LeRobotVLARecorder()
    
    # Terminal setup for non-blocking input
    if not sys.stdin.isatty():
        node.get_logger().error('No interactive terminal. Run in a separate terminal.')
        node.destroy_node()
        rclpy.shutdown()
        return
    
    original_settings = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    
    try:
        while rclpy.ok():
            # Check keyboard input
            if select.select([sys.stdin], [], [], 0.02)[0]:
                key = sys.stdin.read(1).lower()
                
                if key == 'r':
                    node.start_recording()
                elif key == 's':
                    node.stop_and_save_episode()
                elif key == 'q' or key == '\x1b' or key == '\x03':
                    node.finalize_dataset()
                    break
            
            rclpy.spin_once(node, timeout_sec=0.01)
            
    except KeyboardInterrupt:
        node.finalize_dataset()
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, original_settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
