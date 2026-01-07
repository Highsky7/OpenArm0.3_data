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
from sensor_msgs.msg import JointState


class KeyboardGripperController(Node):
    """
    Keyboard-based gripper controller.
    
    Publishes to /left_gripper_cmd and /right_gripper_cmd topics
    with Float64 values (0.0 = open, 1.0 = closed).
    
    Subscribes to /gripper_states to sync initial position with Arduino bridge.
    """
    
    def __init__(self):
        super().__init__('keyboard_gripper_controller')
        
        # Parameters
        self.declare_parameter('gripper_speed', 1.0)  # Position change per second
        self.declare_parameter('publish_rate', 20.0)  # Hz
        self.declare_parameter('min_gripper', 0.0)
        self.declare_parameter('max_gripper', 1.0)
        self.declare_parameter('sync_timeout', 5.0)  # Seconds to wait for sync
        
        self.gripper_speed = self.get_parameter('gripper_speed').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.min_gripper = self.get_parameter('min_gripper').value
        self.max_gripper = self.get_parameter('max_gripper').value
        self.sync_timeout = self.get_parameter('sync_timeout').value
        
        # Current gripper positions (None until synced with Arduino bridge)
        self.left_gripper_pos = None
        self.right_gripper_pos = None
        self.initialized = False
        self.init_start_time = self.get_clock().now()
        
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
        
        # Subscriber for gripper state sync (from Arduino bridge)
        self.gripper_state_sub = self.create_subscription(
            JointState, '/gripper_states', self.gripper_state_callback, 10)
        
        # Timer for continuous publishing
        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)
        
        self.get_logger().info('=== Keyboard Gripper Controller ===')
        self.get_logger().info('  Waiting for gripper state sync from Arduino bridge...')
        self.get_logger().info("  'q' = Left open,  'w' = Left close")
        self.get_logger().info("  'o' = Right open, 'p' = Right close")
        self.get_logger().info("  ESC or Ctrl+C to quit")
    
    def gripper_state_callback(self, msg: JointState):
        """Sync gripper position with Arduino bridge on first message."""
        if self.initialized:
            return  # Already initialized, ignore further updates
        
        # Extract positions from gripper_states message
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                if 'left' in name.lower():
                    self.left_gripper_pos = msg.position[i]
                elif 'right' in name.lower():
                    self.right_gripper_pos = msg.position[i]
        
        # Check if both are synced
        if self.left_gripper_pos is not None and self.right_gripper_pos is not None:
            self.initialized = True
            self.get_logger().info(
                f'✅ Synced with Arduino bridge: L={self.left_gripper_pos:.3f}, R={self.right_gripper_pos:.3f}')
            self.get_logger().info('  Ready for keyboard control!')
    
    def timer_callback(self):
        """Update gripper positions based on key states."""
        # Check initialization state
        if not self.initialized:
            # Check for timeout - fallback to default values
            elapsed = (self.get_clock().now() - self.init_start_time).nanoseconds / 1e9
            if elapsed > self.sync_timeout:
                self.get_logger().warn(
                    f'Sync timeout ({self.sync_timeout}s). Using default position 0.0 (open)')
                self.left_gripper_pos = 0.0
                self.right_gripper_pos = 0.0
                self.initialized = True
            return  # Don't publish until initialized
        
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
