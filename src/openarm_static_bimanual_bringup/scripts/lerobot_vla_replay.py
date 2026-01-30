#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
LeRobot VLA Data Replay for OpenArm Bimanual Robot

Replays recorded trajectories from LeRobot parquet datasets.

Features:
- Load parquet datasets created by lerobot_vla_recorder.py
- Episode selection and playback speed control
- Publishes to forward_position_controller and gripper controllers

Usage:
    ros2 run openarm_static_bimanual_bringup lerobot_vla_replay.py \\
        --ros-args -p dataset_path:=~/lerobot_datasets/openarm_bimanual

Parameters:
    - dataset_path: Path to LeRobot dataset directory
    - episode_index: Episode to replay (-1 for all, default: 0)
    - playback_speed: Speed multiplier (default: 1.0)
    - loop: Loop playback (default: false)

Required Controllers:
    - left_forward_position_controller (active)
    - right_forward_position_controller (active)
    - left_gripper_controller (active)
    - right_gripper_controller (active)
"""
import os
import time
from pathlib import Path
from typing import Optional

import numpy as np
import pyarrow.parquet as pq
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class LeRobotVLAReplay(Node):
    """
    LeRobot VLA Data Replay Node
    
    Loads parquet datasets and replays joint trajectories on the robot.
    """
    
    # Joint ordering matching recorder
    JOINT_NAMES = [
        'left_rev1', 'left_rev2', 'left_rev3', 'left_rev4',
        'left_rev5', 'left_rev6', 'left_rev7', 'left_rev8',
        'right_rev1', 'right_rev2', 'right_rev3', 'right_rev4',
        'right_rev5', 'right_rev6', 'right_rev7', 'right_rev8',
    ]
    
    # Arm joints (excluding gripper)
    LEFT_ARM_JOINTS = ['left_rev1', 'left_rev2', 'left_rev3', 'left_rev4',
                       'left_rev5', 'left_rev6', 'left_rev7']
    RIGHT_ARM_JOINTS = ['right_rev1', 'right_rev2', 'right_rev3', 'right_rev4',
                        'right_rev5', 'right_rev6', 'right_rev7']
    
    def __init__(self):
        super().__init__('lerobot_vla_replay')
        
        # Parameters
        self.declare_parameter('dataset_path', '')
        self.declare_parameter('episode_index', 0)  # -1 for all episodes
        self.declare_parameter('playback_speed', 1.0)
        self.declare_parameter('loop', False)
        self.declare_parameter('use_action', False)  # Use action instead of observation
        
        dataset_path = self.get_parameter('dataset_path').value
        self.episode_index = self.get_parameter('episode_index').value
        self.playback_speed = self.get_parameter('playback_speed').value
        self.loop = self.get_parameter('loop').value
        self.use_action = self.get_parameter('use_action').value
        
        if not dataset_path:
            self.get_logger().error('No dataset_path specified!')
            self.get_logger().error('Usage: --ros-args -p dataset_path:=~/lerobot_datasets/openarm_bimanual')
            return
        
        self.dataset_path = Path(os.path.expanduser(dataset_path))
        
        # Publishers - Use forward_position_controller for direct position commands
        self.left_arm_pub = self.create_publisher(
            Float64MultiArray,
            '/left_forward_position_controller/commands',
            10
        )
        self.right_arm_pub = self.create_publisher(
            Float64MultiArray,
            '/right_forward_position_controller/commands',
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
        
        # Load and play
        self.load_and_play()
    
    def load_and_play(self):
        """Load dataset and start playback."""
        # Find parquet file
        data_dir = self.dataset_path / 'data'
        parquet_files = list(data_dir.glob('*.parquet'))
        
        if not parquet_files:
            self.get_logger().error(f'No parquet files found in {data_dir}')
            return
        
        # Load parquet
        parquet_path = parquet_files[0]
        self.get_logger().info(f'Loading: {parquet_path}')
        
        table = pq.read_table(parquet_path)
        df = table.to_pandas()
        
        self.get_logger().info('=' * 50)
        self.get_logger().info('  LeRobot VLA Replay')
        self.get_logger().info('=' * 50)
        self.get_logger().info(f'  Dataset: {self.dataset_path}')
        self.get_logger().info(f'  Total frames: {len(df)}')
        self.get_logger().info(f'  Episodes: {df["episode_index"].nunique()}')
        self.get_logger().info(f'  Playback speed: {self.playback_speed}x')
        self.get_logger().info(f'  Loop: {self.loop}')
        self.get_logger().info('=' * 50)
        
        # Filter by episode if specified
        if self.episode_index >= 0:
            df = df[df['episode_index'] == self.episode_index]
            if len(df) == 0:
                self.get_logger().error(f'Episode {self.episode_index} not found')
                return
            self.get_logger().info(f'  Playing episode {self.episode_index}: {len(df)} frames')
        
        self.play_trajectory(df)
    
    def play_trajectory(self, df):
        """Play the loaded trajectory."""
        data_key = 'action' if self.use_action else 'observation.state'
        
        while rclpy.ok():
            start_time = time.time()
            prev_timestamp = None
            
            for idx, row in df.iterrows():
                if not rclpy.ok():
                    break
                
                # Get positions
                positions = np.array(row[data_key])
                timestamp = row['timestamp']
                
                # Timing
                if prev_timestamp is not None:
                    dt = (timestamp - prev_timestamp) / self.playback_speed
                    time.sleep(max(0, dt))
                
                prev_timestamp = timestamp
                
                # Extract arm and gripper positions
                left_arm_pos = positions[0:7].tolist()
                left_gripper_pos = float(positions[7])
                right_arm_pos = positions[8:15].tolist()
                right_gripper_pos = float(positions[15])
                
                # Publish arm position commands (using forward_position_controller)
                self._publish_arm_command(self.left_arm_pub, left_arm_pos)
                self._publish_arm_command(self.right_arm_pub, right_arm_pos)
                
                # Publish gripper commands
                self._publish_gripper(self.left_gripper_pub, left_gripper_pos)
                self._publish_gripper(self.right_gripper_pub, right_gripper_pos)
                
                rclpy.spin_once(self, timeout_sec=0.001)
            
            if not self.loop:
                self.get_logger().info('✅ Playback complete')
                break
            else:
                self.get_logger().info('🔄 Looping...')
    
    def _publish_arm_command(self, publisher, positions: list):
        """Publish arm position command using Float64MultiArray."""
        msg = Float64MultiArray()
        msg.data = positions
        publisher.publish(msg)
    
    def _publish_gripper(self, publisher, position: float):
        """Publish gripper command."""
        msg = Float64MultiArray()
        msg.data = [position]
        publisher.publish(msg)
    
    def _publish_gripper(self, publisher, position: float):
        """Publish gripper command."""
        msg = Float64MultiArray()
        msg.data = [position]
        publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LeRobotVLAReplay()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Replay stopped')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
