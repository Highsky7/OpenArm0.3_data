#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
LeRobot VLA Data Recorder for OpenArm Bimanual Robot

Records bimanual robot joint states in LeRobot v3.0 parquet format
for VLA (Vision-Language-Action) model training.

Features:
- 16-DOF recording: left_rev1~8 + right_rev1~8 (including grippers)
- Episode-based recording with keyboard controls
- LeRobot compatible parquet output

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
from datetime import datetime
from pathlib import Path
from typing import Optional

import numpy as np
import pyarrow as pa
import pyarrow.parquet as pq
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class LeRobotVLARecorder(Node):
    """
    LeRobot VLA Data Recorder
    
    Records joint states at a fixed frequency and saves them in
    LeRobot v3.0 compatible parquet format.
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
        super().__init__('lerobot_vla_recorder')
        
        # Parameters
        self.declare_parameter('record_rate', 50.0)  # Hz
        self.declare_parameter('dataset_name', 'openarm_bimanual')
        self.declare_parameter('save_dir', '~/lerobot_datasets')
        self.declare_parameter('robot_type', 'openarm_static_bimanual')
        
        self.record_rate = self.get_parameter('record_rate').value
        self.dataset_name = self.get_parameter('dataset_name').value
        self.save_dir = os.path.expanduser(self.get_parameter('save_dir').value)
        self.robot_type = self.get_parameter('robot_type').value
        
        # Dataset path
        self.dataset_path = Path(self.save_dir) / self.dataset_name
        self.data_dir = self.dataset_path / 'data'
        
        # State
        self.is_recording = False
        self.episode_frames: list[dict] = []
        self.episode_index = 0
        self.start_time: Optional[float] = None
        self.last_joint_state: Optional[JointState] = None
        
        # All episodes data for final parquet
        self.all_frames: list[dict] = []
        
        # Subscriber
        self.js_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10)
        
        # Recording timer
        self.record_timer = self.create_timer(
            1.0 / self.record_rate, self.record_callback)
        
        self._print_banner()
    
    def _print_banner(self):
        """Print startup information."""
        self.get_logger().info('=' * 50)
        self.get_logger().info('  LeRobot VLA Recorder for OpenArm Bimanual')
        self.get_logger().info('=' * 50)
        self.get_logger().info(f'  Record rate: {self.record_rate} Hz')
        self.get_logger().info(f'  Dataset: {self.dataset_path}')
        self.get_logger().info(f'  Joints: {self.NUM_JOINTS} DOF')
        self.get_logger().info('')
        self.get_logger().info('  Keyboard Controls:')
        self.get_logger().info("    'r' - Start new episode")
        self.get_logger().info("    's' - Stop and save episode")
        self.get_logger().info("    'q' - Finalize and quit")
        self.get_logger().info('=' * 50)
        self.get_logger().info('  Waiting for /joint_states...')
    
    def joint_state_callback(self, msg: JointState):
        """Store latest joint state."""
        self.last_joint_state = msg
    
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
    
    def record_callback(self):
        """Record current frame if recording is active."""
        if not self.is_recording or self.last_joint_state is None:
            return
        
        positions = self._get_joint_positions()
        if positions is None:
            return
        
        timestamp = time.time() - self.start_time
        frame_index = len(self.episode_frames)
        
        frame = {
            'timestamp': timestamp,
            'frame_index': frame_index,
            'episode_index': self.episode_index,
            'observation.state': positions.copy(),
            # Action will be filled with next frame's position (for VLA)
            '_positions': positions.copy(),  # Temporary for action computation
        }
        
        self.episode_frames.append(frame)
    
    def start_recording(self):
        """Start new episode recording."""
        if self.last_joint_state is None:
            self.get_logger().warn('Cannot start: No joint_states received yet')
            return
        
        self.episode_frames = []
        self.start_time = time.time()
        self.is_recording = True
        
        self.get_logger().info(f'🔴 Episode {self.episode_index} recording started!')
    
    def stop_and_save_episode(self):
        """Stop recording and save current episode."""
        self.is_recording = False
        
        if len(self.episode_frames) < 2:
            self.get_logger().warn('Episode too short (< 2 frames), discarding')
            self.episode_frames = []
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
        
        duration = self.episode_frames[-1]['timestamp']
        num_frames = len(self.episode_frames)
        
        self.get_logger().info(
            f'💾 Episode {self.episode_index} saved: {num_frames} frames, {duration:.1f}s')
        
        self.episode_index += 1
        self.episode_frames = []
    
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
        
        # Create table with LeRobot v3.0 compatible schema
        table = pa.table({
            'timestamp': pa.array(timestamps, type=pa.float64()),
            'frame_index': pa.array(frame_indices, type=pa.int64()),
            'episode_index': pa.array(episode_indices, type=pa.int64()),
            'observation.state': [obs.tolist() for obs in observations],
            'action': [act.tolist() for act in actions],
        })
        
        # Write parquet file
        parquet_path = self.data_dir / 'train-00000.parquet'
        pq.write_table(table, parquet_path)
        
        # Write metadata (info.json)
        self._write_info_json()
        
        self.get_logger().info('=' * 50)
        self.get_logger().info('✅ Dataset finalized!')
        self.get_logger().info(f'   Path: {self.dataset_path}')
        self.get_logger().info(f'   Episodes: {self.episode_index}')
        self.get_logger().info(f'   Total frames: {len(self.all_frames)}')
        self.get_logger().info('=' * 50)
    
    def _write_info_json(self):
        """Write LeRobot compatible info.json metadata."""
        import json
        
        info = {
            'repo_id': f'local/{self.dataset_name}',
            'robot_type': self.robot_type,
            'fps': int(self.record_rate),
            'num_episodes': self.episode_index,
            'num_frames': len(self.all_frames),
            'features': {
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
            },
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
