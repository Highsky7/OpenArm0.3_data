#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
LeRobot VLA Data Recorder for OpenArm Bimanual Robot

Records bimanual robot joint states and camera images using LeRobot's
official LeRobotDataset API for VLA (Vision-Language-Action) model training.

Features:
- 16-DOF recording: left_rev1~8 + right_rev1~8 (including grippers)
- 3 camera streams: top, wrist_left, wrist_right (256x256)
- Language instruction for VLA training
- Episode-based recording with keyboard controls
- LeRobot v3.0 compatible output with resume support
- Multi-task dataset support

Usage:
    ros2 run openarm_static_bimanual_bringup lerobot_vla_recorder.py

Keyboard Controls:
    'r' - Start new episode recording
    's' - Stop and save current episode
    'q' - Finalize dataset and quit
"""
import os
import select
import sys
import termios
import time
import tty
from pathlib import Path
from threading import Lock
from typing import Optional

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState

# LeRobot imports
from lerobot.datasets.lerobot_dataset import LeRobotDataset


class LeRobotVLARecorder(Node):
    """
    LeRobot VLA Data Recorder using official LeRobotDataset API.
    
    Records joint states and camera images at a fixed frequency and saves them
    using LeRobotDataset's add_frame() and save_episode() methods.
    Supports resume and multi-task recording.
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
        'top': '/camera/cam_1/color/image_raw',
        'wrist_left': '/camera/cam_2/color/image_raw',
        'wrist_right': '/camera/cam_3/color/image_raw',
    }
    IMAGE_SIZE = (256, 256)  # Resize target
    
    def __init__(self):
        super().__init__('lerobot_vla_recorder')
        
        # Parameters
        self.declare_parameter('record_rate', 20.0)  # Hz
        self.declare_parameter('dataset_name', 'openarm_bimanual')
        self.declare_parameter('save_dir', '~/lerobot_datasets')
        self.declare_parameter('robot_type', 'openarm_static_bimanual')
        self.declare_parameter('task_description', 'bimanual manipulation task')
        self.declare_parameter('enable_cameras', True)
        self.declare_parameter('resume', True)  # Resume from existing dataset
        
        self.record_rate = self.get_parameter('record_rate').value
        self.dataset_name = self.get_parameter('dataset_name').value
        self.save_dir = os.path.expanduser(self.get_parameter('save_dir').value)
        self.robot_type = self.get_parameter('robot_type').value
        self.task_description = self.get_parameter('task_description').value
        self.enable_cameras = self.get_parameter('enable_cameras').value
        self.resume = self.get_parameter('resume').value
        
        # Dataset path
        self.dataset_path = Path(self.save_dir) / self.dataset_name
        self.repo_id = f"local/{self.dataset_name}"
        
        # State
        self.is_recording = False
        self.episode_buffer: list[dict] = []  # Buffer for current episode
        self.start_time: Optional[float] = None
        self.last_joint_state: Optional[JointState] = None
        
        # Camera state
        self.cv_bridge = CvBridge()
        self.latest_images: dict[str, Optional[np.ndarray]] = {
            key: None for key in self.CAMERA_TOPICS.keys()
        }
        self.image_lock = Lock()
        
        # Initialize LeRobotDataset
        self.dataset: Optional[LeRobotDataset] = None
        self._init_dataset()
        
        # Subscriber: Joint states
        self.js_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10)
        
        # Subscribers: Cameras (if enabled)
        if self.enable_cameras:
            for key, topic in self.CAMERA_TOPICS.items():
                self.create_subscription(
                    Image,
                    topic,
                    lambda msg, k=key: self.image_callback(msg, k),
                    10
                )
                self.get_logger().info(f'Subscribed: {topic} -> observation.images.{key}')
        
        # Recording timer
        self.record_timer = self.create_timer(
            1.0 / self.record_rate, self.record_callback)
        
        self._print_banner()
    
    def _get_features(self) -> dict:
        """Define dataset features for LeRobotDataset."""
        features = {
            "observation.state": {
                "dtype": "float32",
                "shape": (self.NUM_JOINTS,),
                "names": self.JOINT_NAMES,
            },
            "action": {
                "dtype": "float32",
                "shape": (self.NUM_JOINTS,),
                "names": self.JOINT_NAMES,
            },
        }
        
        # Add camera features if enabled
        if self.enable_cameras:
            for key in self.CAMERA_TOPICS.keys():
                features[f"observation.images.{key}"] = {
                    "dtype": "video",
                    "shape": (self.IMAGE_SIZE[1], self.IMAGE_SIZE[0], 3),  # H, W, C
                    "names": ["height", "width", "channels"],
                }
        
        return features
    
    def _init_dataset(self):
        """Initialize LeRobotDataset with resume support."""
        try:
            if self.resume and self.dataset_path.exists():
                # Load existing dataset (resume mode)
                self.dataset = LeRobotDataset(
                    self.repo_id,
                    root=self.dataset_path,
                )
                self.get_logger().info(
                    f'📂 Resumed dataset: {self.dataset.num_episodes} episodes, '
                    f'{self.dataset.num_frames} frames')
            else:
                # Create new dataset
                features = self._get_features()
                self.dataset = LeRobotDataset.create(
                    repo_id=self.repo_id,
                    fps=int(self.record_rate),
                    root=self.dataset_path,
                    robot_type=self.robot_type,
                    features=features,
                    use_videos=self.enable_cameras,
                )
                self.get_logger().info(f'📁 Created new dataset: {self.dataset_path}')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize dataset: {e}')
            raise
    
    def _print_banner(self):
        """Print startup information."""
        num_episodes = self.dataset.num_episodes if self.dataset else 0
        
        self.get_logger().info('=' * 55)
        self.get_logger().info('  LeRobot VLA Recorder for OpenArm Bimanual')
        self.get_logger().info('=' * 55)
        self.get_logger().info(f'  Record rate: {self.record_rate} Hz')
        self.get_logger().info(f'  Dataset: {self.dataset_path}')
        self.get_logger().info(f'  Existing episodes: {num_episodes}')
        self.get_logger().info(f'  Joints: {self.NUM_JOINTS} DOF')
        self.get_logger().info(f'  Cameras: {"Enabled" if self.enable_cameras else "Disabled"}')
        self.get_logger().info(f'  Image size: {self.IMAGE_SIZE[0]}x{self.IMAGE_SIZE[1]}')
        self.get_logger().info(f'  Task: {self.task_description}')
        self.get_logger().info(f'  Resume: {self.resume}')
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
        
        # Build frame for episode buffer
        frame = {
            'observation.state': positions.copy(),
            '_positions': positions.copy(),  # Temporary for action computation
        }
        
        # Add images
        if self.enable_cameras:
            for key, img in current_images.items():
                frame[f'observation.images.{key}'] = img
        
        self.episode_buffer.append(frame)
    
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
        
        self.episode_buffer = []
        self.start_time = time.time()
        self.is_recording = True
        
        episode_num = self.dataset.num_episodes if self.dataset else 0
        self.get_logger().info(f'🔴 Episode {episode_num} recording started!')
    
    def stop_and_save_episode(self):
        """Stop recording and save current episode using LeRobotDataset API."""
        self.is_recording = False
        
        if len(self.episode_buffer) < 2:
            self.get_logger().warn('Episode too short (< 2 frames), discarding')
            self.episode_buffer = []
            return
        
        # Compute actions (next frame's state as action - absolute position)
        for i in range(len(self.episode_buffer) - 1):
            next_positions = self.episode_buffer[i + 1]['_positions']
            self.episode_buffer[i]['action'] = next_positions
        
        # Last frame uses its own position as action
        self.episode_buffer[-1]['action'] = self.episode_buffer[-1]['_positions']
        
        # Add frames to LeRobotDataset
        for frame in self.episode_buffer:
            # Remove temporary field
            del frame['_positions']
            # Add task description
            frame['task'] = self.task_description
            # Add frame using LeRobotDataset API
            self.dataset.add_frame(frame)
        
        # Save episode using LeRobotDataset API (handles video encoding, stats, metadata)
        self.dataset.save_episode()
        
        duration = time.time() - self.start_time
        num_frames = len(self.episode_buffer)
        episode_num = self.dataset.num_episodes - 1
        
        self.get_logger().info(
            f'💾 Episode {episode_num} saved: {num_frames} frames, {duration:.1f}s')
        
        self.episode_buffer = []
    
    def finalize_dataset(self):
        """Finalize the LeRobot dataset."""
        if self.is_recording:
            self.stop_and_save_episode()
        
        if self.dataset is None:
            self.get_logger().warn('No dataset to finalize')
            return
        
        # Finalize dataset (close parquet writers, write metadata)
        self.dataset.finalize()
        
        self.get_logger().info('=' * 55)
        self.get_logger().info('✅ Dataset finalized!')
        self.get_logger().info(f'   Path: {self.dataset_path}')
        self.get_logger().info(f'   Episodes: {self.dataset.num_episodes}')
        self.get_logger().info(f'   Total frames: {self.dataset.num_frames}')
        self.get_logger().info('=' * 55)


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
