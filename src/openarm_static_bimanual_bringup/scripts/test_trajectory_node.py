#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import math
import time

class TestTrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('test_trajectory_node')

        self.declare_parameter('left_controller_name', 'left_joint_trajectory_controller')
        self.declare_parameter('right_controller_name', 'right_joint_trajectory_controller')
        self.declare_parameter('joint_names', ['left_rev1', 'left_rev2', 'left_rev3', 'left_rev4', 'left_rev5', 'left_rev6', 'left_rev7',
                                               'right_rev1', 'right_rev2', 'right_rev3', 'right_rev4', 'right_rev5', 'right_rev6', 'right_rev7'])
        self.declare_parameter('trajectory_type', 'sine_wave') # Only sine_wave supported for now
        self.declare_parameter('amplitude', 0.5)
        self.declare_parameter('frequency', 0.1) # Hz
        self.declare_parameter('offset', 0.0)
        self.declare_parameter('duration', 5.0) # Duration of one trajectory point in seconds

        self.left_controller_name = self.get_parameter('left_controller_name').value
        self.right_controller_name = self.get_parameter('right_controller_name').value
        self.joint_names = self.get_parameter('joint_names').value
        self.trajectory_type = self.get_parameter('trajectory_type').value
        self.amplitude = self.get_parameter('amplitude').value
        self.frequency = self.get_parameter('frequency').value
        self.offset = self.get_parameter('offset').value
        self.duration = self.get_parameter('duration').value

        self.left_publisher = self.create_publisher(JointTrajectory, f'/{self.left_controller_name}/joint_trajectory', 10)
        self.right_publisher = self.create_publisher(JointTrajectory, f'/{self.right_controller_name}/joint_trajectory', 10)

        self.timer = self.create_timer(1.0 / (self.frequency * 20), self.publish_trajectory) # Publish at 20x frequency for smoother motion
        self.start_time = self.get_clock().now().seconds_nanoseconds()[0]

        self.get_logger().info(f'TestTrajectoryPublisher started. Publishing to /{self.left_controller_name}/joint_trajectory and /{self.right_controller_name}/joint_trajectory')

    def publish_trajectory(self):
        current_time = self.get_clock().now().seconds_nanoseconds()[0] - self.start_time
        
        # Generate sine wave for all joints
        positions = [self.amplitude * math.sin(2 * math.pi * self.frequency * current_time) + self.offset for _ in self.joint_names]
        
        # Create JointTrajectory message
        trajectory_msg = JointTrajectory()
        trajectory_msg.header.stamp = self.get_clock().now().to_msg()
        trajectory_msg.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = int(self.duration)
        point.time_from_start.nanosec = int((self.duration - int(self.duration)) * 1e9)
        trajectory_msg.points.append(point)

        # Split joints for left and right arm
        left_joint_names = [name for name in self.joint_names if name.startswith('left_')]
        right_joint_names = [name for name in self.joint_names if name.startswith('right_')]

        left_positions = positions[:len(left_joint_names)]
        right_positions = positions[len(left_joint_names):]

        # Publish for left arm
        left_trajectory_msg = JointTrajectory()
        left_trajectory_msg.header.stamp = trajectory_msg.header.stamp
        left_trajectory_msg.joint_names = left_joint_names
        left_point = JointTrajectoryPoint()
        left_point.positions = left_positions
        left_point.time_from_start = point.time_from_start
        left_trajectory_msg.points.append(left_point)
        self.left_publisher.publish(left_trajectory_msg)

        # Publish for right arm
        right_trajectory_msg = JointTrajectory()
        right_trajectory_msg.header.stamp = trajectory_msg.header.stamp
        right_trajectory_msg.joint_names = right_joint_names
        right_point = JointTrajectoryPoint()
        right_point.positions = right_positions
        right_point.time_from_start = point.time_from_start
        right_trajectory_msg.points.append(right_point)
        self.right_publisher.publish(right_trajectory_msg)


def main(args=None):
    rclpy.init(args=args)
    node = TestTrajectoryPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
