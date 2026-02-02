#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Simple State Replay for OpenArm Bimanual Robot

Replays recorded trajectory data (observation.state) from LeRobot dataset format.
This is a simplified version that only replays robot motion without camera recording.

Algorithm (from lerobot_vla_replay_recorder.py):
1. Load parquet file(s) containing trajectory data
2. Extract observation.state (16 joint positions) for each frame
3. Publish commands at original trajectory fps with precise timing
4. Use absolute timing to prevent drift accumulation

Usage:
    ros2 run openarm_static_bimanual_bringup simple_state_replay.py \\
        --ros-args \\
        -p dataset_path:=~/lerobot_datasets/openarm_trajectory \\
        -p episode_index:=0 \\
        -p playback_speed:=1.0

Joint Order (16 joints total):
    [0-6]   left_rev1 ~ left_rev7   (left arm)
    [7]     left_rev8               (left gripper)
    [8-14]  right_rev1 ~ right_rev7 (right arm)
    [15]    right_rev8              (right gripper)
"""
import json
import os
import time
from pathlib import Path
from typing import Optional

import numpy as np
import pandas as pd
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class SimpleStateReplay(Node):
    """
    Simple trajectory replay node.
    
    Reads observation.state from LeRobot dataset and publishes position commands
    to gravity compensation node and gripper controllers.
    """
    
    # Joint names for reference
    JOINT_NAMES = [
        'left_rev1', 'left_rev2', 'left_rev3', 'left_rev4',
        'left_rev5', 'left_rev6', 'left_rev7', 'left_rev8',
        'right_rev1', 'right_rev2', 'right_rev3', 'right_rev4',
        'right_rev5', 'right_rev6', 'right_rev7', 'right_rev8',
    ]
    
    def __init__(self):
        super().__init__('simple_state_replay')
        
        # ========== Parameters ==========
        self.declare_parameter('dataset_path', '')
        self.declare_parameter('episode_index', -1)  # -1 = all episodes
        self.declare_parameter('playback_speed', 1.0)  # 1.0 = original speed
        self.declare_parameter('loop', False)  # Loop replay
        self.declare_parameter('start_delay', 3.0)  # Delay before starting
        
        dataset_path = self.get_parameter('dataset_path').value
        self.episode_index = self.get_parameter('episode_index').value
        self.playback_speed = self.get_parameter('playback_speed').value
        self.loop = self.get_parameter('loop').value
        self.start_delay = self.get_parameter('start_delay').value
        
        if not dataset_path:
            self.get_logger().error('❌ No dataset_path specified!')
            self.get_logger().error('   Usage: -p dataset_path:=~/lerobot_datasets/your_dataset')
            return
        
        self.dataset_path = Path(os.path.expanduser(dataset_path))
        
        # ========== Load trajectory fps from info.json ==========
        self.trajectory_fps = self._load_trajectory_fps()
        
        # ========== Publishers ==========
        # Arm position commands (gravity compensation mode)
        self.left_arm_pub = self.create_publisher(
            Float64MultiArray,
            '/gravity_comp/left_external_position_cmd',
            10
        )
        self.right_arm_pub = self.create_publisher(
            Float64MultiArray,
            '/gravity_comp/right_external_position_cmd',
            10
        )
        
        # Gripper position commands
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
        
        # ========== Subscriber for initial position ==========
        self.current_positions: Optional[np.ndarray] = None
        self.js_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10)
        
        # ========== Load trajectory data ==========
        self.trajectory_df = self._load_trajectory_data()
        
        if self.trajectory_df is None or len(self.trajectory_df) == 0:
            self.get_logger().error('❌ Failed to load trajectory data!')
            return
        
        # ========== Start replay after delay ==========
        self.get_logger().info('=' * 60)
        self.get_logger().info('  Simple State Replay')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'  Dataset: {self.dataset_path}')
        self.get_logger().info(f'  Trajectory FPS: {self.trajectory_fps} Hz')
        self.get_logger().info(f'  Playback speed: {self.playback_speed}x')
        self.get_logger().info(f'  Effective FPS: {self.trajectory_fps * self.playback_speed} Hz')
        self.get_logger().info(f'  Total frames: {len(self.trajectory_df)}')
        self.get_logger().info(f'  Loop: {self.loop}')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'  Starting in {self.start_delay} seconds...')
        
        # Timer to start replay
        self.create_timer(self.start_delay, self._start_replay_once)
        self._started = False
    
    def _load_trajectory_fps(self) -> float:
        """Load original trajectory fps from info.json."""
        info_path = self.dataset_path / 'meta' / 'info.json'
        
        if info_path.exists():
            try:
                with open(info_path, 'r') as f:
                    info = json.load(f)
                fps = float(info.get('fps', 30.0))
                self.get_logger().info(f'📁 Loaded fps from info.json: {fps} Hz')
                return fps
            except Exception as e:
                self.get_logger().warn(f'Failed to load info.json: {e}')
        
        self.get_logger().warn('Using default fps: 30.0 Hz')
        return 30.0
    
    def _load_trajectory_data(self) -> Optional[pd.DataFrame]:
        """
        Load trajectory data from parquet files.
        
        Supports LeRobot v3.0 multi-chunk format:
        - data/chunk-000/episode_000000.parquet
        - data/chunk-000/episode_000001.parquet
        - ...
        """
        data_dir = self.dataset_path / 'data'
        
        if not data_dir.exists():
            self.get_logger().error(f'Data directory not found: {data_dir}')
            return None
        
        # Find all parquet files (supports nested chunk directories)
        parquet_files = sorted(data_dir.glob('**/*.parquet'))
        
        if not parquet_files:
            self.get_logger().error(f'No parquet files found in {data_dir}')
            return None
        
        self.get_logger().info(f'📂 Found {len(parquet_files)} parquet file(s)')
        
        # Load and concatenate all parquet files
        dataframes = []
        for pf in parquet_files:
            try:
                df = pd.read_parquet(pf)
                dataframes.append(df)
                self.get_logger().info(f'   Loaded: {pf.name} ({len(df)} frames)')
            except Exception as e:
                self.get_logger().warn(f'   Failed to load {pf}: {e}')
        
        if not dataframes:
            return None
        
        # Concatenate and sort by episode_index, frame_index
        full_df = pd.concat(dataframes, ignore_index=True)
        full_df = full_df.sort_values(['episode_index', 'frame_index']).reset_index(drop=True)
        
        # Filter by episode_index if specified
        if self.episode_index >= 0:
            full_df = full_df[full_df['episode_index'] == self.episode_index]
            self.get_logger().info(f'📌 Filtered to episode {self.episode_index}: {len(full_df)} frames')
        
        return full_df
    
    def joint_state_callback(self, msg: JointState):
        """Store current joint positions for reference."""
        positions = np.zeros(16)
        for i, name in enumerate(msg.name):
            if name in self.JOINT_NAMES and i < len(msg.position):
                idx = self.JOINT_NAMES.index(name)
                positions[idx] = msg.position[i]
        self.current_positions = positions
    
    def _publish_arm_command(self, publisher, positions: np.ndarray):
        """Publish arm position command (7 joints)."""
        msg = Float64MultiArray()
        msg.data = [float(p) for p in positions]
        publisher.publish(msg)
    
    def _publish_gripper_command(self, publisher, position: float):
        """Publish gripper position command (1 joint)."""
        msg = Float64MultiArray()
        msg.data = [float(position)]
        publisher.publish(msg)
    
    def _start_replay_once(self):
        """Start replay only once."""
        if self._started:
            return
        self._started = True
        
        self.get_logger().info('🚀 Starting replay!')
        self._replay_trajectory()
    
    def _replay_trajectory(self):
        """
        Main replay loop with precise timing.
        
        Key algorithm points:
        1. Calculate absolute target time for each frame
        2. Wait until target time (prevents drift accumulation)
        3. Use observation.state for accurate position playback
        """
        df = self.trajectory_df
        trajectory_period = 1.0 / self.trajectory_fps  # seconds per frame
        
        while rclpy.ok():
            episodes = sorted(df['episode_index'].unique())
            
            for ep_idx in episodes:
                if not rclpy.ok():
                    break
                
                ep_df = df[df['episode_index'] == ep_idx].copy()
                ep_df = ep_df.sort_values('frame_index').reset_index(drop=True)
                
                total_frames = len(ep_df)
                duration = total_frames * trajectory_period / self.playback_speed
                
                self.get_logger().info(f'🎬 Episode {ep_idx}: {total_frames} frames, ~{duration:.1f}s')
                
                # ========== Absolute timing for precise playback ==========
                start_time = time.monotonic()
                
                for frame_idx, (_, row) in enumerate(ep_df.iterrows()):
                    if not rclpy.ok():
                        break
                    
                    # Calculate absolute target time for this frame
                    # This prevents timing drift from sleep inaccuracy
                    target_time = start_time + (frame_idx * trajectory_period / self.playback_speed)
                    
                    # ========== Extract positions from observation.state ==========
                    # observation.state contains actual recorded positions (16 values)
                    if 'observation.state' in row and row['observation.state'] is not None:
                        target_positions = np.array(row['observation.state'])
                    elif 'action' in row:
                        # Fallback to action if observation.state not available
                        target_positions = np.array(row['action'])
                    else:
                        self.get_logger().warn(f'No position data at frame {frame_idx}')
                        continue
                    
                    # ========== Parse joint positions ==========
                    # [0:7]  = left arm (rev1~rev7)
                    # [7]    = left gripper (rev8)
                    # [8:15] = right arm (rev1~rev7)
                    # [15]   = right gripper (rev8)
                    left_arm_pos = target_positions[:7]
                    left_gripper_pos = target_positions[7]
                    right_arm_pos = target_positions[8:15]
                    right_gripper_pos = target_positions[15]
                    
                    # ========== Publish commands ==========
                    self._publish_arm_command(self.left_arm_pub, left_arm_pos)
                    self._publish_arm_command(self.right_arm_pub, right_arm_pos)
                    self._publish_gripper_command(self.left_gripper_pub, left_gripper_pos)
                    self._publish_gripper_command(self.right_gripper_pub, right_gripper_pos)
                    
                    # ========== Wait until target time ==========
                    current_time = time.monotonic()
                    wait_time = target_time - current_time
                    if wait_time > 0:
                        time.sleep(wait_time)
                    
                    # ========== Progress logging ==========
                    if frame_idx % 100 == 0:
                        elapsed = time.monotonic() - start_time
                        expected = frame_idx * trajectory_period / self.playback_speed
                        drift_ms = (elapsed - expected) * 1000
                        progress = (frame_idx + 1) / total_frames * 100
                        self.get_logger().info(
                            f'  Frame {frame_idx}/{total_frames} ({progress:.1f}%) | '
                            f'Drift: {drift_ms:+.1f}ms')
                
                self.get_logger().info(f'✅ Episode {ep_idx} complete!')
            
            # Check loop option
            if not self.loop:
                break
            
            self.get_logger().info('🔄 Looping replay...')
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('🏁 Replay finished!')
        self.get_logger().info('=' * 60)


def main(args=None):
    rclpy.init(args=args)
    node = SimpleStateReplay()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
