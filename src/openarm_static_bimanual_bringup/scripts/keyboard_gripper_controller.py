#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Keyboard Gripper Controller for OpenArm Bimanual Robot (CAN Motor Version)

Controls CAN motor grippers via ros2_control:
  - 'q'/'w': Left gripper open/close (decrease/increase position)
  - 'o'/'p': Right gripper open/close (decrease/increase position)

Run in a separate terminal:
  ros2 run openarm_static_bimanual_bringup keyboard_gripper_controller.py
"""
import select
import sys
import termios
import tty

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState


class KeyboardGripperController(Node):
    """
    Keyboard-based gripper controller for CAN motors.
    
    Publishes to /left_gripper_controller/commands and /right_gripper_controller/commands
    for gripper joints (rev8) with Float64MultiArray values in radians.
    
    IMPORTANT: Waits for joint_states to sync actual gripper positions before sending commands.
    """
    
    def __init__(self):
        super().__init__('keyboard_gripper_controller')
        
        # Parameters
        self.declare_parameter('gripper_speed', 2.0)  # rad/s
        self.declare_parameter('publish_rate', 20.0)  # Hz
        # Gripper limits - extended range to handle motor zero offset
        # URDF says 0.0~1.57, but motor may have offset from calibration
        self.declare_parameter('min_gripper', -0.5)   # Extended for motor offset
        self.declare_parameter('max_gripper', 2.0)    # Extended for motor offset
        self.declare_parameter('left_joint_name', 'left_rev8')
        self.declare_parameter('right_joint_name', 'right_rev8')
        
        self.gripper_speed = self.get_parameter('gripper_speed').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.min_gripper = self.get_parameter('min_gripper').value
        self.max_gripper = self.get_parameter('max_gripper').value
        self.left_joint_name = self.get_parameter('left_joint_name').value
        self.right_joint_name = self.get_parameter('right_joint_name').value
        
        # Current gripper positions - None until synced from joint_states
        self.left_gripper_pos = None
        self.right_gripper_pos = None
        self.initialized = False
        
        # Key states (for continuous movement while held)
        self.keys_pressed = {
            'q': False,  # Left open (decrease)
            'w': False,  # Left close (increase)
            'o': False,  # Right open (decrease)
            'p': False,  # Right close (increase)
        }
        
        # Publishers for individual gripper commands
        self.left_gripper_pub = self.create_publisher(
            Float64MultiArray, '/left_gripper_controller/commands', 10)
        self.right_gripper_pub = self.create_publisher(
            Float64MultiArray, '/right_gripper_controller/commands', 10)
        
        # Subscribe to joint_states to get current gripper positions
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10)
        
        # Timer for continuous publishing
        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)
        
        self.get_logger().info('=== Keyboard Gripper Controller (CAN Motor) ===')
        self.get_logger().info(f'  Left joint: {self.left_joint_name}')
        self.get_logger().info(f'  Right joint: {self.right_joint_name}')
        self.get_logger().info('  Gripper range: %.2f ~ %.2f rad' % (self.min_gripper, self.max_gripper))
        self.get_logger().info("  'q' = Left open,  'w' = Left close")
        self.get_logger().info("  'o' = Right open, 'p' = Right close")
        self.get_logger().info("  ESC or Ctrl+C to quit")
        self.get_logger().info('  Waiting for joint_states to sync...')
    
    def joint_state_callback(self, msg: JointState):
        """Update current gripper positions from joint states."""
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                if name == self.left_joint_name:
                    if self.left_gripper_pos is None:
                        self.get_logger().info(f'  ✓ Left gripper synced: {msg.position[i]:.3f} rad')
                    self.left_gripper_pos = msg.position[i]
                elif name == self.right_joint_name:
                    if self.right_gripper_pos is None:
                        self.get_logger().info(f'  ✓ Right gripper synced: {msg.position[i]:.3f} rad')
                    self.right_gripper_pos = msg.position[i]
        
        # Check initialization
        if not self.initialized:
            if self.left_gripper_pos is not None and self.right_gripper_pos is not None:
                self.initialized = True
                self.get_logger().info('  ✓ Both grippers synced! Ready for keyboard control.')
            elif self.left_gripper_pos is not None or self.right_gripper_pos is not None:
                # Log available joints for debugging
                if not hasattr(self, '_logged_joints'):
                    self._logged_joints = True
                    self.get_logger().warn(f'  Available joints: {list(msg.name)}')
    
    def timer_callback(self):
        """Update gripper positions based on key states."""
        # Don't publish until we've synced with actual positions
        if not self.initialized:
            return
        
        dt = 1.0 / self.publish_rate
        delta = self.gripper_speed * dt
        
        # Left gripper
        if self.keys_pressed['q']:  # Open (decrease position)
            self.left_gripper_pos = max(self.min_gripper, self.left_gripper_pos - delta)
        if self.keys_pressed['w']:  # Close (increase position)
            self.left_gripper_pos = min(self.max_gripper, self.left_gripper_pos + delta)
        
        # Right gripper
        if self.keys_pressed['o']:  # Open (decrease position)
            self.right_gripper_pos = max(self.min_gripper, self.right_gripper_pos - delta)
        if self.keys_pressed['p']:  # Close (increase position)
            self.right_gripper_pos = min(self.max_gripper, self.right_gripper_pos + delta)
        
        # Publish gripper commands
        left_msg = Float64MultiArray()
        left_msg.data = [self.left_gripper_pos]
        self.left_gripper_pub.publish(left_msg)
        
        right_msg = Float64MultiArray()
        right_msg.data = [self.right_gripper_pos]
        self.right_gripper_pub.publish(right_msg)
    
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
            # Check for keyboard input with small timeout
            if select.select([sys.stdin], [], [], 0.02)[0]:
                key = sys.stdin.read(1)
                
                if key == '\x1b':  # ESC
                    break
                elif key == '\x03':  # Ctrl+C
                    break
                elif key.lower() in node.keys_pressed:
                    # Set key pressed - will be processed in timer_callback
                    node.handle_key(key.lower(), True)
                    # Log key press for debugging
                    node.get_logger().info(f"Key '{key.lower()}' pressed - L:{node.left_gripper_pos:.3f} R:{node.right_gripper_pos:.3f}")
            
            # Process ROS callbacks
            rclpy.spin_once(node, timeout_sec=0.01)
            
            # Reset keys after processing (they get one update cycle)
            for k in list(node.keys_pressed.keys()):
                node.keys_pressed[k] = False
                
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, original_settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
