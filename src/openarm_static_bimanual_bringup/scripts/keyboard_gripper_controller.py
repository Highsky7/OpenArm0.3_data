#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Keyboard Gripper Controller for OpenArm Bimanual Robot

Controls grippers via keyboard input:
  - 'q'/'w': Left gripper open/close
  - 'o'/'p': Right gripper open/close

Run in a separate terminal:
  ros2 run openarm_static_bimanual_bringup keyboard_gripper_controller.py
"""
import select
import sys
import termios
import tty

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


class KeyboardGripperController(Node):
    """
    Keyboard-based gripper controller.
    
    Publishes to /left_gripper_cmd and /right_gripper_cmd topics
    with Float64 values (0.0 = open, 1.0 = closed).
    """
    
    def __init__(self):
        super().__init__('keyboard_gripper_controller')
        
        # Parameters
        self.declare_parameter('gripper_speed', 0.5)  # Position change per second
        self.declare_parameter('publish_rate', 20.0)  # Hz
        self.declare_parameter('min_gripper', 0.0)
        self.declare_parameter('max_gripper', 1.0)
        
        self.gripper_speed = self.get_parameter('gripper_speed').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.min_gripper = self.get_parameter('min_gripper').value
        self.max_gripper = self.get_parameter('max_gripper').value
        
        # Current gripper positions
        self.left_gripper_pos = 0.5
        self.right_gripper_pos = 0.5
        
        # Key states (for continuous movement while held)
        self.keys_pressed = {
            'q': False,  # Left open
            'w': False,  # Left close
            'o': False,  # Right open
            'p': False,  # Right close
        }
        
        # Publishers
        self.left_pub = self.create_publisher(Float64, '/left_gripper_cmd', 10)
        self.right_pub = self.create_publisher(Float64, '/right_gripper_cmd', 10)
        
        # Timer for continuous publishing
        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)
        
        self.get_logger().info('=== Keyboard Gripper Controller ===')
        self.get_logger().info("  'q' = Left open,  'w' = Left close")
        self.get_logger().info("  'o' = Right open, 'p' = Right close")
        self.get_logger().info("  ESC or Ctrl+C to quit")
    
    def timer_callback(self):
        """Update gripper positions based on key states."""
        dt = 1.0 / self.publish_rate
        delta = self.gripper_speed * dt
        
        # Left gripper
        if self.keys_pressed['q']:
            self.left_gripper_pos = max(self.min_gripper, self.left_gripper_pos - delta)
        if self.keys_pressed['w']:
            self.left_gripper_pos = min(self.max_gripper, self.left_gripper_pos + delta)
        
        # Right gripper
        if self.keys_pressed['o']:
            self.right_gripper_pos = max(self.min_gripper, self.right_gripper_pos - delta)
        if self.keys_pressed['p']:
            self.right_gripper_pos = min(self.max_gripper, self.right_gripper_pos + delta)
        
        # Publish
        left_msg = Float64()
        left_msg.data = self.left_gripper_pos
        self.left_pub.publish(left_msg)
        
        right_msg = Float64()
        right_msg.data = self.right_gripper_pos
        self.right_pub.publish(right_msg)
    
    def handle_key(self, key: str, pressed: bool):
        """Handle key press/release events."""
        if key in self.keys_pressed:
            self.keys_pressed[key] = pressed


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardGripperController()
    
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
            # Check for keyboard input
            if select.select([sys.stdin], [], [], 0.0)[0]:
                key = sys.stdin.read(1)
                
                if key == '\x1b':  # ESC
                    break
                elif key == '\x03':  # Ctrl+C
                    break
                elif key.lower() in node.keys_pressed:
                    # Toggle key state
                    node.handle_key(key.lower(), True)
            else:
                # Reset all keys when no input
                for k in node.keys_pressed:
                    node.keys_pressed[k] = False
            
            rclpy.spin_once(node, timeout_sec=0.01)
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, original_settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
