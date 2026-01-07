#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import json
import os
import time
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

class JointTrajectoryPlayer(Node):
    def __init__(self):
        super().__init__('joint_trajectory_player')

        string_array_descriptor = ParameterDescriptor(type=ParameterType.PARAMETER_STRING_ARRAY)

        self.declare_parameter('waypoint_file', 'recorded_waypoints.json')
        self.declare_parameter('left_controller_name', 'left_joint_trajectory_controller')
        self.declare_parameter('right_controller_name', 'right_joint_trajectory_controller')
        self.declare_parameter('left_joint_names', ['left_rev1', 'left_rev2', 'left_rev3', 'left_rev4', 'left_rev5', 'left_rev6', 'left_rev7'], string_array_descriptor)
        self.declare_parameter('right_joint_names', ['right_rev1', 'right_rev2', 'right_rev3', 'right_rev4', 'right_rev5', 'right_rev6', 'right_rev7'], string_array_descriptor)
        self.declare_parameter('waypoint_interval', 2.0) # Default to 2.0 seconds

        self.waypoint_file = self.get_parameter('waypoint_file').value
        self.left_controller_name = self.get_parameter('left_controller_name').value
        self.right_controller_name = self.get_parameter('right_controller_name').value
        self.canonical_left_joint_names = self.get_parameter('left_joint_names').value
        self.canonical_right_joint_names = self.get_parameter('right_joint_names').value
        self.waypoint_interval = self.get_parameter('waypoint_interval').value

        self.left_publisher = self.create_publisher(JointTrajectory, f'/{self.left_controller_name}/joint_trajectory', 10)
        self.right_publisher = self.create_publisher(JointTrajectory, f'/{self.right_controller_name}/joint_trajectory', 10)

        self.recorded_waypoints = []
        self.load_waypoints()

        if not self.recorded_waypoints:
            self.get_logger().error('No waypoints loaded. Exiting.')
            return

        self.get_logger().info('JointTrajectoryPlayer node started. Playing back trajectory.')
        self.play_trajectory()

    def load_waypoints(self):
        filepath = os.path.join(os.path.expanduser('~'), 'ros2_ws', self.waypoint_file) # Load from workspace root
        if not os.path.exists(filepath):
            self.get_logger().error(f'Waypoint file not found: {filepath}')
            return

        with open(filepath, 'r') as f:
            self.recorded_waypoints = json.load(f)
        self.get_logger().info(f'Loaded {len(self.recorded_waypoints)} waypoints from {filepath}')

    def play_trajectory(self):
        if not self.recorded_waypoints:
            self.get_logger().warn('No waypoints to play.')
            return

        left_trajectory_msg = JointTrajectory()
        left_trajectory_msg.header.stamp = self.get_clock().now().to_msg()
        left_trajectory_msg.joint_names = self.canonical_left_joint_names

        right_trajectory_msg = JointTrajectory()
        right_trajectory_msg.header.stamp = self.get_clock().now().to_msg()
        right_trajectory_msg.joint_names = self.canonical_right_joint_names

        cumulative_time = 0.0
        for waypoint_data in self.recorded_waypoints:
            cumulative_time += self.waypoint_interval

            recorded_joint_names = waypoint_data["joint_names"]
            recorded_positions = waypoint_data["positions"]
            recorded_velocities = waypoint_data["velocities"]

            recorded_name_to_index = {name: i for i, name in enumerate(recorded_joint_names)}

            left_positions = [0.0] * len(self.canonical_left_joint_names)
            left_velocities = [0.0] * len(self.canonical_left_joint_names)
            for i, name in enumerate(self.canonical_left_joint_names):
                if name in recorded_name_to_index:
                    idx = recorded_name_to_index[name]
                    left_positions[i] = recorded_positions[idx]
                    if recorded_velocities: 
                        left_velocities[i] = recorded_velocities[idx]
                else:
                    self.get_logger().warn(f'Joint {name} not found in recorded data for left arm. Using 0.0.')

            right_positions = [0.0] * len(self.canonical_right_joint_names)
            right_velocities = [0.0] * len(self.canonical_right_joint_names)
            for i, name in enumerate(self.canonical_right_joint_names):
                if name in recorded_name_to_index:
                    idx = recorded_name_to_index[name]
                    right_positions[i] = recorded_positions[idx]
                    if recorded_velocities: 
                        right_velocities[i] = recorded_velocities[idx]
                else:
                    self.get_logger().warn(f'Joint {name} not found in recorded data for right arm. Using 0.0.')

            left_point = JointTrajectoryPoint()
            left_point.positions = left_positions
            left_point.velocities = left_velocities
            left_point.time_from_start = Duration(sec=int(cumulative_time), nanosec=int((cumulative_time - int(cumulative_time)) * 1e9))
            left_trajectory_msg.points.append(left_point)

            right_point = JointTrajectoryPoint()
            right_point.positions = right_positions
            right_point.velocities = right_velocities
            right_point.time_from_start = Duration(sec=int(cumulative_time), nanosec=int((cumulative_time - int(cumulative_time)) * 1e9))
            right_trajectory_msg.points.append(right_point)
        
        self.left_publisher.publish(left_trajectory_msg)
        self.right_publisher.publish(right_trajectory_msg)
        self.get_logger().info(f'Published trajectory with {len(self.recorded_waypoints)} waypoints.')


def main(args=None):
    rclpy.init(args=args)
    node = JointTrajectoryPlayer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
