#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
LeRobot VLA Replay Recorder for OpenArm Bimanual Robot (Phase 2)

Replays trajectory from Phase 1 dataset while recording camera observations.
This creates complete VLA datasets with observation + action data.

Features:
- Load trajectory dataset from Phase 1
- Replay with gravity compensation mode
- Record camera observations during replay
- Generate complete VLA dataset for model training

Usage:
    ros2 run openarm_static_bimanual_bringup lerobot_vla_replay_recorder.py \\
        --ros-args \\
        -p trajectory_dataset:=~/lerobot_datasets/openarm_trajectory \\
        -p vla_dataset:=~/lerobot_datasets/openarm_vla

Part of 2-Phase VLA Data Collection Workflow:
    Phase 1: Manual teaching -> trajectory_dataset (lerobot_trajectory_recorder.py)
    Phase 2: Replay + record -> vla_dataset (this script)
"""
import os
import time
from pathlib import Path
from threading import Lock
from typing import Optional

import cv2
import numpy as np
import pandas as pd
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Float64MultiArray

# LeRobot imports
from lerobot.datasets.lerobot_dataset import LeRobotDataset


class LeRobotVLAReplayRecorder(Node):
    """
    LeRobot VLA Replay Recorder - Phase 2 of 2-Phase VLA Collection.
    
    Replays a trajectory dataset while recording camera observations
    to create a complete VLA dataset for model training.
    """
    
    # Joint ordering
    JOINT_NAMES = [
        'left_rev1', 'left_rev2', 'left_rev3', 'left_rev4',
        'left_rev5', 'left_rev6', 'left_rev7', 'left_rev8',
        'right_rev1', 'right_rev2', 'right_rev3', 'right_rev4',
        'right_rev5', 'right_rev6', 'right_rev7', 'right_rev8',
    ]
    NUM_JOINTS = 16
    
    # Arm joints (excluding gripper)
    LEFT_ARM_JOINTS = ['left_rev1', 'left_rev2', 'left_rev3', 'left_rev4',
                       'left_rev5', 'left_rev6', 'left_rev7']
    RIGHT_ARM_JOINTS = ['right_rev1', 'right_rev2', 'right_rev3', 'right_rev4',
                        'right_rev5', 'right_rev6', 'right_rev7']
    
    # Camera configuration
    CAMERA_TOPICS = {
        'top': '/camera/cam_1/color/image_raw',
        'wrist_left': '/camera/cam_2/color/image_raw',
        'wrist_right': '/camera/cam_3/color/image_raw',
    }
    IMAGE_SIZE = (256, 256)
    
    def __init__(self):
        super().__init__('lerobot_vla_replay_recorder')
        
        # Parameters
        self.declare_parameter('trajectory_dataset', '')
        self.declare_parameter('vla_dataset', '')
        self.declare_parameter('episode_index', -1)  # -1 for all episodes
        self.declare_parameter('playback_speed', 1.0)
        self.declare_parameter('record_rate', 20.0)  # Hz
        self.declare_parameter('robot_type', 'openarm_static_bimanual')
        self.declare_parameter('task_description', 'bimanual manipulation task')
        self.declare_parameter('resume', True)
        
        trajectory_path = self.get_parameter('trajectory_dataset').value
        vla_path = self.get_parameter('vla_dataset').value
        self.episode_index = self.get_parameter('episode_index').value
        self.playback_speed = self.get_parameter('playback_speed').value
        self.record_rate = self.get_parameter('record_rate').value
        self.robot_type = self.get_parameter('robot_type').value
        self.task_description = self.get_parameter('task_description').value
        self.resume = self.get_parameter('resume').value
        
        if not trajectory_path:
            self.get_logger().error('No trajectory_dataset specified!')
            return
        
        if not vla_path:
            # Default: trajectory path + "_vla"
            vla_path = trajectory_path.rstrip('/') + '_vla'
        
        self.trajectory_path = Path(os.path.expanduser(trajectory_path))
        self.vla_path = Path(os.path.expanduser(vla_path))
        self.vla_repo_id = f"local/{self.vla_path.name}"
        
        # State
        self.last_joint_state: Optional[JointState] = None
        self.cv_bridge = CvBridge()
        self.latest_images: dict[str, Optional[np.ndarray]] = {
            key: None for key in self.CAMERA_TOPICS.keys()
        }
        self.image_lock = Lock()
        
        # VLA Dataset
        self.vla_dataset: Optional[LeRobotDataset] = None
        
        # Publishers for gravity compensation mode
        self.left_gravity_comp_pub = self.create_publisher(
            Float64MultiArray,
            '/gravity_comp/left_external_position_cmd',
            10
        )
        self.right_gravity_comp_pub = self.create_publisher(
            Float64MultiArray,
            '/gravity_comp/right_external_position_cmd',
            10
        )
        self.left_gripper_pub = self.create_publisher(
            Float64MultiArray,
            '/left_gripper_controller/commands',
            10
        )
        self.right_gripper_pub = self.create_publisher(
            Float64MultiArray,
            '/right_gripper_controller/commands',
            10
        )
        
        # Subscriber: Joint states
        self.js_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10)
        
        # Subscribers: Cameras
        for key, topic in self.CAMERA_TOPICS.items():
            self.create_subscription(
                Image,
                topic,
                lambda msg, k=key: self.image_callback(msg, k),
                10
            )
            self.get_logger().info(f'Subscribed: {topic} -> observation.images.{key}')
        
        # Initialize VLA dataset
        self._init_vla_dataset()
        
        # Start replay+record
        self.get_logger().info('Waiting for cameras and joint states...')
        self.create_timer(2.0, self.start_replay_once)
        self._started = False
    
    def _get_features(self) -> dict:
        """Define VLA dataset features with cameras."""
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
        
        # Add camera features
        for key in self.CAMERA_TOPICS.keys():
            features[f"observation.images.{key}"] = {
                "dtype": "video",
                "shape": (self.IMAGE_SIZE[1], self.IMAGE_SIZE[0], 3),
                "names": ["height", "width", "channels"],
            }
        
        return features
    
    def _init_vla_dataset(self):
        """Initialize VLA dataset."""
        try:
            if self.resume and self.vla_path.exists():
                self.vla_dataset = LeRobotDataset(
                    self.vla_repo_id,
                    root=self.vla_path,
                )
                self.get_logger().info(
                    f'📂 Resumed VLA dataset: {self.vla_dataset.num_episodes} episodes')
            else:
                features = self._get_features()
                self.vla_dataset = LeRobotDataset.create(
                    repo_id=self.vla_repo_id,
                    fps=int(self.record_rate),
                    root=self.vla_path,
                    robot_type=self.robot_type,
                    features=features,
                    use_videos=True,
                )
                self.get_logger().info(f'📁 Created new VLA dataset: {self.vla_path}')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize VLA dataset: {e}')
            raise
    
    def joint_state_callback(self, msg: JointState):
        """Store latest joint state."""
        self.last_joint_state = msg
    
    def image_callback(self, msg: Image, image_key: str):
        """Store and resize latest camera image."""
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, 'rgb8')
            resized = cv2.resize(cv_image, self.IMAGE_SIZE, interpolation=cv2.INTER_AREA)
            with self.image_lock:
                self.latest_images[image_key] = resized
        except Exception as e:
            self.get_logger().warn(f'Image conversion error ({image_key}): {e}')
    
    def _get_current_images(self) -> dict[str, Optional[np.ndarray]]:
        """Get current images with thread safety."""
        with self.image_lock:
            return {k: v.copy() if v is not None else None 
                    for k, v in self.latest_images.items()}
    
    def _get_joint_positions(self) -> Optional[np.ndarray]:
        """Extract joint positions in correct order."""
        if self.last_joint_state is None:
            return None
        
        pos_dict = {}
        for i, name in enumerate(self.last_joint_state.name):
            if i < len(self.last_joint_state.position):
                pos_dict[name] = self.last_joint_state.position[i]
        
        positions = []
        for joint_name in self.JOINT_NAMES:
            if joint_name in pos_dict:
                positions.append(pos_dict[joint_name])
            else:
                positions.append(0.0)
        
        return np.array(positions, dtype=np.float32)
    
    def _publish_arm_command(self, publisher, positions):
        """Publish arm position command."""
        msg = Float64MultiArray()
        msg.data = list(positions)
        publisher.publish(msg)
    
    def _publish_gripper_command(self, publisher, position):
        """Publish gripper position command."""
        msg = Float64MultiArray()
        msg.data = [float(position)]
        publisher.publish(msg)
    
    def start_replay_once(self):
        """Start replay once cameras are ready."""
        if self._started:
            return
        
        # Check cameras
        current_images = self._get_current_images()
        missing = [k for k, v in current_images.items() if v is None]
        if missing:
            self.get_logger().info(f'Waiting for cameras: {missing}')
            return
        
        if self.last_joint_state is None:
            self.get_logger().info('Waiting for joint states...')
            return
        
        self._started = True
        self.replay_and_record()
    
    def replay_and_record(self):
        """Main replay and record loop."""
        # Load trajectory dataset (supports multi-chunk LeRobot v3.0 format)
        data_dir = self.trajectory_path / 'data'
        parquet_files = sorted(data_dir.glob('**/*.parquet'))
        
        if not parquet_files:
            self.get_logger().error(f'No parquet files in {data_dir}')
            return
        
        # Load and concatenate all parquet files for multi-chunk support
        dataframes = [pd.read_parquet(f) for f in parquet_files]
        df = pd.concat(dataframes, ignore_index=True)
        df = df.sort_values(['episode_index', 'frame_index']).reset_index(drop=True)
        
        self.get_logger().info(f'Loaded {len(parquet_files)} parquet file(s), {len(df)} total frames')
        
        # Get episodes to process
        if self.episode_index >= 0:
            episodes = [self.episode_index]
        else:
            episodes = sorted(df['episode_index'].unique())
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('  Phase 2: VLA Replay + Record')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'  Trajectory source: {self.trajectory_path}')
        self.get_logger().info(f'  VLA output: {self.vla_path}')
        self.get_logger().info(f'  Episodes to process: {len(episodes)}')
        self.get_logger().info(f'  Record rate: {self.record_rate} Hz')
        self.get_logger().info('=' * 60)
        
        # Process each episode
        for ep_idx in episodes:
            if not rclpy.ok():
                break
            
            ep_df = df[df['episode_index'] == ep_idx].copy()
            ep_df = ep_df.sort_values('frame_index').reset_index(drop=True)
            
            self.get_logger().info(f'🎬 Processing episode {ep_idx}: {len(ep_df)} frames')
            self._process_episode(ep_df, ep_idx)
        
        # Finalize
        self.vla_dataset.finalize()
        self.get_logger().info('=' * 60)
        self.get_logger().info('✅ VLA dataset finalized!')
        self.get_logger().info(f'   Path: {self.vla_path}')
        self.get_logger().info(f'   Episodes: {self.vla_dataset.num_episodes}')
        self.get_logger().info(f'   Total frames: {self.vla_dataset.num_frames}')
        self.get_logger().info('=' * 60)
    
    def _process_episode(self, ep_df, episode_idx: int):
        """Process a single episode: replay + record."""
        record_period = 1.0 / self.record_rate
        frames_recorded = 0
        
        # Pre-compute actions from trajectory (Option A: copy from trajectory)
        actions = []
        for i in range(len(ep_df)):
            action = np.array(ep_df.iloc[i]['action'])
            actions.append(action)
        
        for idx, row in ep_df.iterrows():
            if not rclpy.ok():
                break
            
            # Get target positions from trajectory action
            target_positions = np.array(row['action'])
            
            # Extract arm and gripper positions
            left_arm_pos = target_positions[:7]
            left_gripper_pos = target_positions[7]
            right_arm_pos = target_positions[8:15]
            right_gripper_pos = target_positions[15]
            
            # Publish commands to gravity compensation node
            self._publish_arm_command(self.left_gravity_comp_pub, left_arm_pos)
            self._publish_arm_command(self.right_gravity_comp_pub, right_arm_pos)
            self._publish_gripper_command(self.left_gripper_pub, left_gripper_pos)
            self._publish_gripper_command(self.right_gripper_pub, right_gripper_pos)
            
            # Small delay for robot to reach position
            time.sleep(record_period * 0.3 / self.playback_speed)
            rclpy.spin_once(self, timeout_sec=0.001)
            
            # Record current state + cameras
            current_positions = self._get_joint_positions()
            current_images = self._get_current_images()
            
            if current_positions is None:
                continue
            
            if any(img is None for img in current_images.values()):
                self.get_logger().warn('Skipping frame: missing camera image')
                continue
            
            # Build VLA frame
            frame = {
                'observation.state': current_positions.copy(),
                'action': target_positions.copy(),  # Option A: use trajectory action
                'task': self.task_description,
            }
            
            # Add camera images
            for key, img in current_images.items():
                frame[f'observation.images.{key}'] = img
            
            self.vla_dataset.add_frame(frame)
            frames_recorded += 1
            
            # Timing for playback speed
            remaining_sleep = record_period * 0.7 / self.playback_speed
            if remaining_sleep > 0:
                time.sleep(remaining_sleep)
            
            rclpy.spin_once(self, timeout_sec=0.001)
        
        # Save episode
        if frames_recorded > 1:
            self.vla_dataset.save_episode()
            self.get_logger().info(f'💾 Episode saved: {frames_recorded} frames')
        else:
            self.get_logger().warn('Episode too short, skipping')


def main(args=None):
    rclpy.init(args=args)
    node = LeRobotVLAReplayRecorder()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node.vla_dataset:
            node.vla_dataset.finalize()
            node.get_logger().info('Dataset finalized on interrupt')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
