#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Trajectory Replay Node for OpenArm Bimanual Robot

Replays recorded JSON trajectories from continuous_recorder_node.

Usage:
  ros2 run openarm_static_bimanual_bringup trajectory_replay_node.py \
      --ros-args -p trajectory_file:=/path/to/trajectory.json
"""
import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


class TrajectoryReplayNode(Node):
    """
    Replays recorded joint trajectories.
    
    Reads JSON files created by continuous_recorder_node and publishes
    to trajectory controllers and gripper command topics.
    """
    
    def __init__(self):
        super().__init__('trajectory_replay_node')
        
        # Parameters
        self.declare_parameter('trajectory_file', '')
        self.declare_parameter('playback_speed', 1.0)
        self.declare_parameter('loop', False)
        
        self.trajectory_file = self.get_parameter('trajectory_file').value
        self.playback_speed = self.get_parameter('playback_speed').value
        self.loop = self.get_parameter('loop').value
        
        # Publishers
        self.left_traj_pub = self.create_publisher(
            JointTrajectory,
            '/left_joint_trajectory_controller/joint_trajectory',
            10
        )
        self.right_traj_pub = self.create_publisher(
            JointTrajectory,
            '/right_joint_trajectory_controller/joint_trajectory',
            10
        )
        self.left_gripper_pub = self.create_publisher(Float64, '/left_gripper_cmd', 10)
        self.right_gripper_pub = self.create_publisher(Float64, '/right_gripper_cmd', 10)
        
        # Load trajectory
        if self.trajectory_file:
            self.load_and_play()
        else:
            self.get_logger().error('No trajectory_file specified! Use -p trajectory_file:=/path/to/file.json')
    
    def load_and_play(self):
        """Load trajectory file and start playback."""
        try:
            with open(self.trajectory_file, 'r') as f:
                data = json.load(f)
        except Exception as e:
            self.get_logger().error(f'Failed to load trajectory: {e}')
            return
        
        if 'data' not in data:
            self.get_logger().error('Invalid trajectory format: missing "data" field')
            return
        
        trajectory_data = data['data']
        metadata = data.get('metadata', {})
        
        self.get_logger().info(f'=== Trajectory Replay ===')
        self.get_logger().info(f'  File: {self.trajectory_file}')
        self.get_logger().info(f'  Samples: {len(trajectory_data)}')
        self.get_logger().info(f'  Duration: {metadata.get("duration_sec", "unknown")}s')
        self.get_logger().info(f'  Playback speed: {self.playback_speed}x')
        
        self.play_trajectory(trajectory_data)
    
    def play_trajectory(self, trajectory_data):
        """Play the loaded trajectory."""
        if not trajectory_data:
            return
        
        # Left arm joints
        left_arm_joints = [f'left_rev{i}' for i in range(1, 8)]
        right_arm_joints = [f'right_rev{i}' for i in range(1, 8)]
        
        start_time = time.time()
        sample_idx = 0
        last_timestamp = 0.0
        
        while rclpy.ok():
            if sample_idx >= len(trajectory_data):
                if self.loop:
                    sample_idx = 0
                    start_time = time.time()
                    self.get_logger().info('Looping trajectory...')
                else:
                    self.get_logger().info('Trajectory playback complete.')
                    break
            
            sample = trajectory_data[sample_idx]
            target_time = sample['timestamp'] / self.playback_speed
            
            # Wait for correct timing
            elapsed = time.time() - start_time
            if elapsed < target_time:
                time.sleep(max(0, target_time - elapsed))
            
            # Build joint position dict
            joint_positions = {}
            for i, name in enumerate(sample.get('joint_names', [])):
                if i < len(sample.get('positions', [])):
                    joint_positions[name] = sample['positions'][i]
            
            # Publish left arm trajectory
            left_positions = [joint_positions.get(j, 0.0) for j in left_arm_joints]
            self.publish_trajectory(self.left_traj_pub, left_arm_joints, left_positions)
            
            # Publish right arm trajectory
            right_positions = [joint_positions.get(j, 0.0) for j in right_arm_joints]
            self.publish_trajectory(self.right_traj_pub, right_arm_joints, right_positions)
            
            # Publish gripper commands
            left_gripper_pos = joint_positions.get('left_gripper_joint', 0.5)
            right_gripper_pos = joint_positions.get('right_gripper_joint', 0.5)
            
            self.publish_gripper(self.left_gripper_pub, left_gripper_pos)
            self.publish_gripper(self.right_gripper_pub, right_gripper_pos)
            
            sample_idx += 1
            rclpy.spin_once(self, timeout_sec=0.001)
    
    def publish_trajectory(self, publisher, joint_names, positions):
        """Publish a trajectory point."""
        msg = JointTrajectory()
        msg.joint_names = joint_names
        
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = Duration(sec=0, nanosec=50_000_000)  # 50ms
        
        msg.points = [point]
        publisher.publish(msg)
    
    def publish_gripper(self, publisher, position):
        """Publish gripper command."""
        msg = Float64()
        msg.data = float(position)
        publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryReplayNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
