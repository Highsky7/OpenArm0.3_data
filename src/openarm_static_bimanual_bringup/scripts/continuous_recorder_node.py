#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Continuous Joint State Recorder for OpenArm Bimanual Robot

Records joint states at a specified frequency for motion learning/playback.
Includes safety features to detect and warn about joint limit violations.
"""
import json
import math
import os
import select
import sys
import termios
import tty
from datetime import datetime

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class ContinuousJointRecorder(Node):
    """
    Continuous Joint State Recorder
    
    Features:
    - Records at configurable frequency (default 50Hz)
    - Saves to NPY or JSON format
    - Joint limit warning during recording
    - Keyboard controls: 'r' start, 's' stop/save, 'q' quit
    """
    
    # Joint limits from URDF (rad)
    JOINT_LIMITS = {
        'rev1': (-3.09, 3.09),    # ±177°
        'rev2': (-2.57, 2.57),    # ±147°
        'rev3': (-3.09, 3.09),    # ±177°
        'rev4': (-0.5, 3.62),      # -28° ~ 207°
        'rev5': (-3.09, 3.09),    # ±177°
        'rev6': (-2.57, 2.57),    # ±147°
        'rev7': (-1.96, 1.96),    # ±112°
        'left_pris1': (-0.05, 0.0),  # Gripper joint (prismatic)
    }
    
    def __init__(self):
        super().__init__('continuous_joint_recorder')
        
        # Parameters
        self.declare_parameter('record_rate', 50.0)  # Hz
        self.declare_parameter('save_dir', os.path.expanduser('~/openarm_official_ws'))
        self.declare_parameter('file_format', 'json')  # 'npy' or 'json'
        self.declare_parameter('enable_limit_warning', True)
        
        self.record_rate = self.get_parameter('record_rate').value
        self.save_dir = self.get_parameter('save_dir').value
        self.file_format = self.get_parameter('file_format').value
        self.enable_limit_warning = self.get_parameter('enable_limit_warning').value
        
        # State
        self.is_recording = False
        self.recorded_data = []
        self.last_joint_state = None
        self.start_time = None
        self.warned_joints = set()  # Track warned joints to avoid spam
        
        # Joint names (arm joints from ros2_control)
        self.left_arm_joints = [f'left_rev{i}' for i in range(1, 8)]
        self.right_arm_joints = [f'right_rev{i}' for i in range(1, 8)]
        
        # Gripper joint names (from Arduino bridge - /gripper_states topic)
        self.left_gripper_joint = 'left_gripper_joint'
        self.right_gripper_joint = 'right_gripper_joint'
        
        # Combined joint lists for recording
        self.left_joints = self.left_arm_joints + [self.left_gripper_joint]
        self.right_joints = self.right_arm_joints + [self.right_gripper_joint]
        
        # Last gripper state (from separate topic)
        self.last_gripper_state = None
        
        # Subscribers
        self.js_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10)
        
        # Gripper states subscriber (Arduino bridge publishes to /gripper_states)
        self.gripper_sub = self.create_subscription(
            JointState, '/gripper_states', self.gripper_state_callback, 10)
        
        # Recording timer
        self.record_timer = self.create_timer(
            1.0 / self.record_rate, self.record_callback)
        
        self.get_logger().info('=== Continuous Joint Recorder Initialized ===')
        self.get_logger().info(f'  Record rate: {self.record_rate} Hz')
        self.get_logger().info(f'  Save directory: {self.save_dir}')
        self.get_logger().info(f'  File format: {self.file_format}')
        self.get_logger().info("  Controls: 'r'=start, 's'=stop/save, 'q'=quit")
    
    def joint_state_callback(self, msg: JointState):
        """Store latest joint state from ros2_control."""
        self.last_joint_state = msg
    
    def gripper_state_callback(self, msg: JointState):
        """Store latest gripper state from Arduino bridge."""
        self.last_gripper_state = msg
    
    def check_joint_limits(self, joint_name: str, position: float) -> bool:
        """
        Check if joint is within safe limits.
        Returns True if OK, False if near/over limit.
        """
        if not self.enable_limit_warning:
            return True
        
        # Extract joint type (e.g., 'rev1' from 'left_rev1')
        joint_type = joint_name.split('_')[-1] if '_' in joint_name else joint_name
        
        if joint_type not in self.JOINT_LIMITS:
            return True
        
        lower, upper = self.JOINT_LIMITS[joint_type]
        margin = 0.087  # ~5° safety margin
        
        if position < lower + margin or position > upper - margin:
            if joint_name not in self.warned_joints:
                self.get_logger().warn(
                    f'⚠️  {joint_name} near limit! pos={math.degrees(position):.1f}° '
                    f'(range: {math.degrees(lower):.0f}° ~ {math.degrees(upper):.0f}°)')
                self.warned_joints.add(joint_name)
            return False
        else:
            # Clear warning when back in safe range
            self.warned_joints.discard(joint_name)
            return True
    
    def record_callback(self):
        """Record current joint state if recording is active."""
        if not self.is_recording or self.last_joint_state is None:
            return
        
        timestamp = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        
        # Combine arm joints and gripper joints
        combined_names = list(self.last_joint_state.name)
        combined_positions = list(self.last_joint_state.position)
        combined_velocities = list(self.last_joint_state.velocity) if self.last_joint_state.velocity else []
        combined_efforts = list(self.last_joint_state.effort) if self.last_joint_state.effort else []
        
        # Add gripper data if available (from Arduino bridge)
        if self.last_gripper_state is not None:
            for i, name in enumerate(self.last_gripper_state.name):
                if name not in combined_names:
                    combined_names.append(name)
                    if i < len(self.last_gripper_state.position):
                        combined_positions.append(self.last_gripper_state.position[i])
                    if self.last_gripper_state.velocity and i < len(self.last_gripper_state.velocity):
                        combined_velocities.append(self.last_gripper_state.velocity[i])
                    else:
                        combined_velocities.append(0.0)
                    # Gripper typically doesn't have effort
                    combined_efforts.append(0.0)
        
        # Check joint limits for arm joints
        for i, name in enumerate(self.last_joint_state.name):
            if i < len(self.last_joint_state.position):
                self.check_joint_limits(name, self.last_joint_state.position[i])
        
        data_point = {
            'timestamp': timestamp,
            'joint_names': combined_names,
            'positions': combined_positions,
            'velocities': combined_velocities,
            'efforts': combined_efforts
        }
        self.recorded_data.append(data_point)
    
    def start_recording(self):
        """Start recording joint states."""
        self.recorded_data = []
        self.start_time = self.get_clock().now()
        self.is_recording = True
        self.warned_joints.clear()
        self.get_logger().info('🔴 Recording started!')
    
    def stop_and_save(self):
        """Stop recording and save to file."""
        self.is_recording = False
        
        if not self.recorded_data:
            self.get_logger().warn('No data recorded')
            return
        
        # Create save directory if needed
        os.makedirs(self.save_dir, exist_ok=True)
        
        timestamp_str = datetime.now().strftime('%Y%m%d_%H%M%S')
        
        if self.file_format == 'npy':
            # Save as numpy array
            positions = np.array([d['positions'] for d in self.recorded_data])
            timestamps = np.array([d['timestamp'] for d in self.recorded_data])
            
            filepath = os.path.join(self.save_dir, f'joint_trajectory_{timestamp_str}.npy')
            np.save(filepath, {
                'timestamps': timestamps,
                'positions': positions,
                'joint_names': self.recorded_data[0]['joint_names']
            }, allow_pickle=True)
        else:
            # Save as JSON
            filepath = os.path.join(self.save_dir, f'joint_trajectory_{timestamp_str}.json')
            with open(filepath, 'w') as f:
                json.dump({
                    'metadata': {
                        'record_rate': self.record_rate,
                        'total_samples': len(self.recorded_data),
                        'duration_sec': self.recorded_data[-1]['timestamp'] if self.recorded_data else 0,
                        'recorded_at': timestamp_str
                    },
                    'data': self.recorded_data
                }, f, indent=2)
        
        duration = self.recorded_data[-1]['timestamp'] if self.recorded_data else 0
        self.get_logger().info(
            f'💾 Saved {len(self.recorded_data)} samples ({duration:.1f}s) to {filepath}')
        self.recorded_data = []


def main(args=None):
    rclpy.init(args=args)
    node = ContinuousJointRecorder()
    
    # Set up terminal for non-blocking input
    original_settings = None
    if sys.stdin.isatty():
        original_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        node.get_logger().info('Interactive terminal detected. Keyboard input enabled.')
    else:
        node.get_logger().warn('Non-interactive terminal. Keyboard input disabled.')
    
    try:
        while rclpy.ok():
            if sys.stdin.isatty() and select.select([sys.stdin], [], [], 0.0)[0]:
                key = sys.stdin.read(1)
                if key == 'r':
                    node.start_recording()
                elif key == 's':
                    node.stop_and_save()
                elif key == 'q' or key == '\x03':  # q or Ctrl+C
                    if node.is_recording:
                        node.stop_and_save()
                    break
            rclpy.spin_once(node, timeout_sec=0.01)
    except KeyboardInterrupt:
        if node.is_recording:
            node.stop_and_save()
        node.get_logger().info('Recorder stopped.')
    finally:
        if original_settings:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, original_settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
