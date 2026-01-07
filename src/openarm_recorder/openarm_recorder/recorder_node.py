#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import json
import os
import sys
import select
import termios
import tty
from datetime import datetime

class JointStateRecorder(Node):
    def __init__(self):
        super().__init__('joint_state_recorder')
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        self.subscription  # prevent unused variable warning
        self.recorded_waypoints = []
        self.is_recording = False
        self.last_joint_state_msg = None # Store the last received JointState message

        self.declare_parameter('waypoint_filename', '') # Allow specifying output filename
        self.waypoint_filename = self.get_parameter('waypoint_filename').value

        self.get_logger().info('JointStateRecorder node started. Press "r" to start recording, "p" to record a waypoint, "q" to stop and save.')

    def joint_state_callback(self, msg):
        # Always store the last received JointState message
        self.last_joint_state_msg = msg

    def record_single_waypoint(self):
        if not self.is_recording:
            self.get_logger().warn('Not recording. Press "r" to start recording first.')
            return
        if self.last_joint_state_msg is None:
            self.get_logger().warn('No JointState message received yet. Cannot record waypoint.')
            return

        # Process the last received JointState message into a waypoint
        waypoint = {
            "header": {
                "stamp": {"sec": self.last_joint_state_msg.header.stamp.sec, "nanosec": self.last_joint_state_msg.header.stamp.nanosec},
                "frame_id": self.last_joint_state_msg.header.frame_id
            },
            "joint_names": self.last_joint_state_msg.name,
            "positions": list(self.last_joint_state_msg.position),
            "velocities": list(self.last_joint_state_msg.velocity),
            "efforts": list(self.last_joint_state_msg.effort)
        }
        self.recorded_waypoints.append(waypoint)
        self.get_logger().info(f'Recorded waypoint: {len(self.recorded_waypoints)}')

    def start_recording(self):
        self.recorded_waypoints = []
        self.is_recording = True
        self.get_logger().info('Recording started.')

    def stop_recording_and_save(self):
        self.is_recording = False
        if not self.recorded_waypoints:
            self.get_logger().warn('No waypoints recorded to save.')
            return

        # Determine filename
        if self.waypoint_filename:
            filename = self.waypoint_filename
        else:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f'recorded_waypoints_{timestamp}.json'

        # Ensure the directory exists
        save_dir = os.path.join(os.path.expanduser('~'), 'ros2_ws') # Save to workspace root
        os.makedirs(save_dir, exist_ok=True)
        filepath = os.path.join(save_dir, filename)

        with open(filepath, 'w') as f:
            json.dump(self.recorded_waypoints, f, indent=4)
        self.get_logger().info(f'Recording stopped. Saved {len(self.recorded_waypoints)} waypoints to {filepath}')
        self.recorded_waypoints = [] # Clear after saving

def main(args=None):
    rclpy.init(args=args)
    node = JointStateRecorder()

    # Set up terminal for non-blocking input only if running in an interactive terminal
    original_settings = None
    if sys.stdin.isatty():
        original_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        node.get_logger().info("Interactive terminal detected. Keyboard input enabled.")
    else:
        node.get_logger().warn("Non-interactive terminal detected. Keyboard input disabled. Node will spin until killed.")

    try:
        while rclpy.ok():
            if sys.stdin.isatty() and select.select([sys.stdin], [], [], 0.0)[0]:
                key = sys.stdin.read(1)
                if key == 'r':
                    node.start_recording()
                elif key == 'p':
                    node.record_single_waypoint()
                elif key == 'q':
                    node.stop_recording_and_save()
                    break # Exit after saving
                elif key == '\x03': # Ctrl+C
                    break
            rclpy.spin_once(node, timeout_sec=0.1)
    except Exception as e:
        node.get_logger().error(f'An error occurred: {e}')
    finally:
        # Restore terminal settings only if they were modified
        if original_settings:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, original_settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
