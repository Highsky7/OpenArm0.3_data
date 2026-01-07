#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
FMVLA Data Recorder for OpenArm Bimanual Robot

Records robot data in LeRobot Dataset v3.0 format (Apache Parquet).
Suitable for VLA (Vision-Language-Action) model training.

Run in a separate terminal:
  ros2 run openarm_static_bimanual_bringup fmvla_data_record.py

Keyboard Controls:
  'r' = Start recording episode
  's' = Stop and save episode
  'f' = Finish dataset
  'q' = Quit
"""
import json
import os
import select
import sys
import termios
import tty
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

# Check for optional dependencies
try:
    import pyarrow as pa
    import pyarrow.parquet as pq
    HAS_PARQUET = True
except ImportError:
    HAS_PARQUET = False


class FMVLADataRecorder(Node):
    """
    FMVLA Data Recorder for LeRobot-compatible format.
    
    Records observation.state and action data in Parquet format.
    Data structure: 16 DOF (left_arm[7] + left_gripper[1] + right_arm[7] + right_gripper[1])
    """
    
    # Fixed joint order for consistent data structure
    JOINT_ORDER = [
        'left_rev1', 'left_rev2', 'left_rev3', 'left_rev4',
        'left_rev5', 'left_rev6', 'left_rev7', 'left_gripper_joint',
        'right_rev1', 'right_rev2', 'right_rev3', 'right_rev4',
        'right_rev5', 'right_rev6', 'right_rev7', 'right_gripper_joint'
    ]
    
    def __init__(self):
        super().__init__('fmvla_data_recorder')
        
        # Parameters
        self.declare_parameter('repo_id', 'openarm/bimanual_dataset')
        self.declare_parameter('root_dir', os.path.expanduser('~/openarm_official_ws/vla_data'))
        self.declare_parameter('fps', 50.0)
        self.declare_parameter('push_to_hub', False)
        
        self.repo_id = self.get_parameter('repo_id').value
        self.root_dir = Path(self.get_parameter('root_dir').value).expanduser()
        self.fps = self.get_parameter('fps').value
        self.push_to_hub = self.get_parameter('push_to_hub').value
        
        # State
        self.is_recording = False
        self.current_episode = []
        self.episodes = []
        self.episode_count = 0
        self.last_joint_state = None
        self.last_gripper_state = None
        self.prev_state = None  # For action computation
        
        # Subscribers
        self.js_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10)
        self.gripper_sub = self.create_subscription(
            JointState, '/gripper_states', self.gripper_state_callback, 10)
        
        # Recording timer
        self.record_timer = self.create_timer(1.0 / self.fps, self.record_callback)
        
        self.get_logger().info('=== FMVLA Data Recorder ===')
        self.get_logger().info(f'  Save directory: {self.root_dir}')
        self.get_logger().info(f'  FPS: {self.fps}')
        self.get_logger().info(f'  Push to Hub: {self.push_to_hub}')
        self.get_logger().info("  Controls: 'r'=record, 's'=save episode, 'f'=finish, 'q'=quit")
        
        if not HAS_PARQUET:
            self.get_logger().warn('pyarrow not installed. Saving to JSON instead.')
    
    def joint_state_callback(self, msg: JointState):
        self.last_joint_state = msg
    
    def gripper_state_callback(self, msg: JointState):
        self.last_gripper_state = msg
    
    def get_current_state(self):
        """Get current state as 16-DOF array in fixed order."""
        if self.last_joint_state is None:
            return None
        
        # Build state dictionary from arm joints
        state_dict = {}
        for i, name in enumerate(self.last_joint_state.name):
            if i < len(self.last_joint_state.position):
                state_dict[name] = self.last_joint_state.position[i]
        
        # Add gripper joints
        if self.last_gripper_state is not None:
            for i, name in enumerate(self.last_gripper_state.name):
                if i < len(self.last_gripper_state.position):
                    state_dict[name] = self.last_gripper_state.position[i]
        
        # Build ordered state array
        state = []
        for joint_name in self.JOINT_ORDER:
            if joint_name in state_dict:
                state.append(float(state_dict[joint_name]))
            else:
                state.append(0.0)  # Default for missing joints
        
        return state
    
    def record_callback(self):
        """Record current state if recording is active."""
        if not self.is_recording:
            return
        
        state = self.get_current_state()
        if state is None:
            return
        
        # Compute action (delta from previous state)
        if self.prev_state is not None:
            action = [s - p for s, p in zip(state, self.prev_state)]
        else:
            action = [0.0] * len(state)
        
        timestamp = self.get_clock().now().nanoseconds / 1e9
        
        data_point = {
            'timestamp': timestamp,
            'frame_index': len(self.current_episode),
            'episode_index': self.episode_count,
            'observation.state': state,
            'action': action,
        }
        
        self.current_episode.append(data_point)
        self.prev_state = state
    
    def start_recording(self):
        """Start recording a new episode."""
        self.current_episode = []
        self.prev_state = None
        self.is_recording = True
        self.get_logger().info(f'🔴 Recording episode {self.episode_count}...')
    
    def save_episode(self):
        """Stop recording and save current episode."""
        self.is_recording = False
        
        if not self.current_episode:
            self.get_logger().warn('No data recorded for this episode.')
            return
        
        self.episodes.append(self.current_episode)
        self.get_logger().info(
            f'💾 Episode {self.episode_count} saved: {len(self.current_episode)} frames')
        
        self.current_episode = []
        self.episode_count += 1
        self.prev_state = None
    
    def finish_dataset(self):
        """Finish and save the complete dataset."""
        if self.is_recording:
            self.save_episode()
        
        if not self.episodes:
            self.get_logger().warn('No episodes recorded.')
            return
        
        # Create output directory
        self.root_dir.mkdir(parents=True, exist_ok=True)
        timestamp_str = datetime.now().strftime('%Y%m%d_%H%M%S')
        
        # Flatten all episodes
        all_data = []
        for ep_idx, episode in enumerate(self.episodes):
            for frame in episode:
                frame['episode_index'] = ep_idx
                all_data.append(frame)
        
        if HAS_PARQUET:
            # Save as Parquet (LeRobot format)
            filepath = self.root_dir / f'dataset_{timestamp_str}.parquet'
            
            table = pa.Table.from_pydict({
                'timestamp': [d['timestamp'] for d in all_data],
                'frame_index': [d['frame_index'] for d in all_data],
                'episode_index': [d['episode_index'] for d in all_data],
                'observation.state': [d['observation.state'] for d in all_data],
                'action': [d['action'] for d in all_data],
            })
            pq.write_table(table, filepath)
            
            # Save metadata
            meta_filepath = self.root_dir / f'meta_{timestamp_str}.json'
            with open(meta_filepath, 'w') as f:
                json.dump({
                    'repo_id': self.repo_id,
                    'fps': self.fps,
                    'total_episodes': len(self.episodes),
                    'total_frames': len(all_data),
                    'joint_order': self.JOINT_ORDER,
                    'codebase_version': 'v3.0',
                }, f, indent=2)
            
            self.get_logger().info(f'✅ Dataset saved: {filepath}')
        else:
            # Fallback to JSON
            filepath = self.root_dir / f'dataset_{timestamp_str}.json'
            with open(filepath, 'w') as f:
                json.dump({
                    'metadata': {
                        'repo_id': self.repo_id,
                        'fps': self.fps,
                        'total_episodes': len(self.episodes),
                        'total_frames': len(all_data),
                        'joint_order': self.JOINT_ORDER,
                    },
                    'data': all_data
                }, f, indent=2)
            
            self.get_logger().info(f'✅ Dataset saved (JSON): {filepath}')
        
        self.episodes = []
        self.episode_count = 0


def main(args=None):
    rclpy.init(args=args)
    node = FMVLADataRecorder()
    
    if not sys.stdin.isatty():
        node.get_logger().error('No interactive terminal. Run in a separate terminal.')
        node.destroy_node()
        rclpy.shutdown()
        return
    
    original_settings = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    
    try:
        while rclpy.ok():
            if select.select([sys.stdin], [], [], 0.0)[0]:
                key = sys.stdin.read(1).lower()
                
                if key == 'r':
                    node.start_recording()
                elif key == 's':
                    node.save_episode()
                elif key == 'f':
                    node.finish_dataset()
                elif key == 'q' or key == '\x03':
                    if node.is_recording:
                        node.save_episode()
                    if node.episodes:
                        node.finish_dataset()
                    break
            
            rclpy.spin_once(node, timeout_sec=0.01)
    except KeyboardInterrupt:
        if node.is_recording:
            node.save_episode()
        if node.episodes:
            node.finish_dataset()
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, original_settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
