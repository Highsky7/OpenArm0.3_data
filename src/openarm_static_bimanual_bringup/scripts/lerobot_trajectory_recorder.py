#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
LeRobot Trajectory Recorder for OpenArm Bimanual Robot (Phase 1)

Records ONLY joint states (no cameras) for trajectory collection.
This creates lightweight trajectory datasets that can be replayed
with lerobot_vla_replay_recorder.py to collect observation data.

Features:
- 16-DOF recording: left_rev1~8 + right_rev1~8 (including grippers)
- NO camera recording (lightweight)
- Higher recording rate possible (30-50Hz)
- Episode-based recording with keyboard controls
- LeRobot v3.0 compatible output

Usage:
    ros2 run openarm_static_bimanual_bringup lerobot_trajectory_recorder.py

Keyboard Controls:
    'r' - Start new episode recording
    's' - Stop and save current episode
    'q' - Finalize dataset and quit

Part of 2-Phase VLA Data Collection Workflow:
    Phase 1: Manual teaching -> trajectory_dataset (this script)
    Phase 2: Replay + record -> vla_dataset (lerobot_vla_replay_recorder.py)
"""
import os
import select
import sys
import termios
import time
import tty
from pathlib import Path
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

# LeRobot imports
from lerobot.datasets.lerobot_dataset import LeRobotDataset


class LeRobotTrajectoryRecorder(Node):
    """
    LeRobot Trajectory Recorder - Phase 1 of 2-Phase VLA Collection.
    
    Records only joint states without camera data for lightweight trajectory
    collection during manual teaching with gravity compensation.
    """
    
    # Joint ordering for dataset (16 DOF total)
    JOINT_NAMES = [
        'left_rev1', 'left_rev2', 'left_rev3', 'left_rev4',
        'left_rev5', 'left_rev6', 'left_rev7', 'left_rev8',
        'right_rev1', 'right_rev2', 'right_rev3', 'right_rev4',
        'right_rev5', 'right_rev6', 'right_rev7', 'right_rev8',
    ]
    NUM_JOINTS = 16
    
    def __init__(self):
        super().__init__('lerobot_trajectory_recorder')
        
        # Parameters
        self.declare_parameter('record_rate', 30.0)  # Hz (higher than VLA since no cameras)
        self.declare_parameter('dataset_name', 'openarm_trajectory')
        self.declare_parameter('save_dir', '~/lerobot_datasets')
        self.declare_parameter('robot_type', 'openarm_static_bimanual')
        self.declare_parameter('task_description', 'bimanual manipulation task')
        self.declare_parameter('resume', True)
        
        self.record_rate = self.get_parameter('record_rate').value
        self.dataset_name = self.get_parameter('dataset_name').value
        self.save_dir = os.path.expanduser(self.get_parameter('save_dir').value)
        self.robot_type = self.get_parameter('robot_type').value
        self.task_description = self.get_parameter('task_description').value
        self.resume = self.get_parameter('resume').value
        
        # Dataset path
        self.dataset_path = Path(self.save_dir) / self.dataset_name
        self.repo_id = f"local/{self.dataset_name}"
        
        # State
        self.is_recording = False
        self.episode_buffer: list[dict] = []
        self.start_time: Optional[float] = None
        self.last_joint_state: Optional[JointState] = None
        
        # Initialize LeRobotDataset
        self.dataset: Optional[LeRobotDataset] = None
        self._init_dataset()
        
        # Subscriber: Joint states
        self.js_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10)
        
        # Recording timer
        self.record_timer = self.create_timer(
            1.0 / self.record_rate, self.record_callback)
        
        self._print_banner()
    
    def _get_features(self) -> dict:
        """Define dataset features - NO cameras, only state and action."""
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
        # NO observation.images.* - this is trajectory-only recording
        return features
    
    def _init_dataset(self):
        """Initialize LeRobotDataset with resume support."""
        try:
            if self.resume and self.dataset_path.exists():
                self.dataset = LeRobotDataset(
                    self.repo_id,
                    root=self.dataset_path,
                )
                self.get_logger().info(
                    f'📂 Resumed trajectory dataset: {self.dataset.num_episodes} episodes, '
                    f'{self.dataset.num_frames} frames')
            else:
                features = self._get_features()
                self.dataset = LeRobotDataset.create(
                    repo_id=self.repo_id,
                    fps=int(self.record_rate),
                    root=self.dataset_path,
                    robot_type=self.robot_type,
                    features=features,
                    use_videos=False,  # No video encoding needed
                )
                self.get_logger().info(f'📁 Created new trajectory dataset: {self.dataset_path}')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize dataset: {e}')
            raise
    
    def _print_banner(self):
        """Print startup information."""
        num_episodes = self.dataset.num_episodes if self.dataset else 0
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('  LeRobot TRAJECTORY Recorder (Phase 1)')
        self.get_logger().info('  For 2-Phase VLA Data Collection')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'  Record rate: {self.record_rate} Hz')
        self.get_logger().info(f'  Dataset: {self.dataset_path}')
        self.get_logger().info(f'  Existing episodes: {num_episodes}')
        self.get_logger().info(f'  Joints: {self.NUM_JOINTS} DOF')
        self.get_logger().info(f'  Cameras: DISABLED (trajectory-only mode)')
        self.get_logger().info(f'  Task: {self.task_description}')
        self.get_logger().info('')
        self.get_logger().info('  Keyboard Controls:')
        self.get_logger().info("    'r' - Start new episode")
        self.get_logger().info("    's' - Stop and save episode")
        self.get_logger().info("    'q' - Finalize and quit")
        self.get_logger().info('=' * 60)
        self.get_logger().info('  Waiting for /joint_states...')
    
    def joint_state_callback(self, msg: JointState):
        """Store latest joint state."""
        self.last_joint_state = msg
    
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
    
    def record_callback(self):
        """Record current frame if recording is active."""
        if not self.is_recording or self.last_joint_state is None:
            return
        
        positions = self._get_joint_positions()
        if positions is None:
            return
        
        # Build frame - NO images
        frame = {
            'observation.state': positions.copy(),
            '_positions': positions.copy(),  # Temporary for action computation
        }
        
        self.episode_buffer.append(frame)
    
    def start_recording(self):
        """Start new episode recording."""
        if self.last_joint_state is None:
            self.get_logger().warn('Cannot start: No joint_states received yet')
            return
        
        self.episode_buffer = []
        self.start_time = time.time()
        self.is_recording = True
        
        episode_num = self.dataset.num_episodes if self.dataset else 0
        self.get_logger().info(f'🔴 Episode {episode_num} recording started!')
    
    def stop_and_save_episode(self):
        """Stop recording and save current episode."""
        self.is_recording = False
        
        if len(self.episode_buffer) < 2:
            self.get_logger().warn('Episode too short (< 2 frames), discarding')
            self.episode_buffer = []
            return
        
        # Compute actions (next frame's position as action)
        for i in range(len(self.episode_buffer) - 1):
            next_positions = self.episode_buffer[i + 1]['_positions']
            self.episode_buffer[i]['action'] = next_positions
        
        # Last frame uses its own position as action
        self.episode_buffer[-1]['action'] = self.episode_buffer[-1]['_positions']
        
        # Add frames to LeRobotDataset
        for frame in self.episode_buffer:
            del frame['_positions']
            frame['task'] = self.task_description
            self.dataset.add_frame(frame)
        
        self.dataset.save_episode()
        
        duration = time.time() - self.start_time
        num_frames = len(self.episode_buffer)
        episode_num = self.dataset.num_episodes - 1
        
        self.get_logger().info(
            f'💾 Episode {episode_num} saved: {num_frames} frames, {duration:.1f}s')
        
        self.episode_buffer = []
    
    def finalize_dataset(self):
        """Finalize the trajectory dataset."""
        if self.is_recording:
            self.stop_and_save_episode()
        
        if self.dataset is None:
            self.get_logger().warn('No dataset to finalize')
            return
        
        self.dataset.finalize()
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('✅ Trajectory dataset finalized!')
        self.get_logger().info(f'   Path: {self.dataset_path}')
        self.get_logger().info(f'   Episodes: {self.dataset.num_episodes}')
        self.get_logger().info(f'   Total frames: {self.dataset.num_frames}')
        self.get_logger().info('')
        self.get_logger().info('   Next step: Run Phase 2 with lerobot_vla_collection.launch.py')
        self.get_logger().info('=' * 60)


def main(args=None):
    rclpy.init(args=args)
    node = LeRobotTrajectoryRecorder()
    
    if not sys.stdin.isatty():
        node.get_logger().error('No interactive terminal. Run in a separate terminal.')
        node.destroy_node()
        rclpy.shutdown()
        return
    
    original_settings = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    
    try:
        while rclpy.ok():
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
