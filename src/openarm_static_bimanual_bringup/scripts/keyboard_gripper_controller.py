#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Keyboard Gripper Controller for OpenArm Bimanual Robot (CAN Motor Version)

Controls CAN motor grippers via ros2_control:
  - 'q'/'w': Left gripper open/close (decrease/increase position)
  - 'o'/'p': Right gripper open/close (decrease/increase position)

Run in a separate terminal:
  ros2 run openarm_static_bimanual_bringup keyboard_gripper_controller.py

FIX: Separated target position from actual position to prevent jittering.
     - target_pos: commanded position (only modified by keyboard input)
     - actual_pos: feedback from joint_states (for monitoring only)
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
    
    IMPORTANT: Separates target position (commanded) from actual position (feedback)
    to prevent jittering caused by feedback overwriting the command.
    """
    
    def __init__(self):
        super().__init__('keyboard_gripper_controller')
        
        # Parameters
        self.declare_parameter('gripper_speed', 4.0)  # rad/s (reduced from 16 for smoother motion)
        self.declare_parameter('publish_rate', 30.0)  # Hz (increased for smoother updates)
        # Gripper limits - extended range to handle motor zero offset
        # URDF says 0.0~1.57, but motor may have offset from calibration
        self.declare_parameter('min_gripper', -0.05)   # Extended for motor offset
        self.declare_parameter('max_gripper', 0.9)    # Extended for motor offset
        self.declare_parameter('left_joint_name', 'left_rev8')
        self.declare_parameter('right_joint_name', 'right_rev8')
        
        self.gripper_speed = self.get_parameter('gripper_speed').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.min_gripper = self.get_parameter('min_gripper').value
        self.max_gripper = self.get_parameter('max_gripper').value
        self.left_joint_name = self.get_parameter('left_joint_name').value
        self.right_joint_name = self.get_parameter('right_joint_name').value
        
        # TARGET positions (commanded) - modified only by keyboard input
        self.left_target_pos = None
        self.right_target_pos = None
        
        # ACTUAL positions (feedback) - updated from joint_states for monitoring
        self.left_actual_pos = None
        self.right_actual_pos = None
        
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
        self.get_logger().info(f'  Speed: {self.gripper_speed} rad/s, Rate: {self.publish_rate} Hz')
        self.get_logger().info("  'q' = Left open,  'w' = Left close")
        self.get_logger().info("  'o' = Right open, 'p' = Right close")
        self.get_logger().info("  ESC or Ctrl+C to quit")
        self.get_logger().info('  Waiting for joint_states to sync...')
    
    def joint_state_callback(self, msg: JointState):
        """Update actual gripper positions from joint states (for monitoring only)."""
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                if name == self.left_joint_name:
                    # First time: initialize TARGET from actual position
                    if self.left_target_pos is None:
                        self.left_target_pos = msg.position[i]
                        self.get_logger().info(f'  ✓ Left gripper synced: {msg.position[i]:.3f} rad')
                    # Always update ACTUAL position (for monitoring only, not for commands)
                    self.left_actual_pos = msg.position[i]
                    
                elif name == self.right_joint_name:
                    # First time: initialize TARGET from actual position
                    if self.right_target_pos is None:
                        self.right_target_pos = msg.position[i]
                        self.get_logger().info(f'  ✓ Right gripper synced: {msg.position[i]:.3f} rad')
                    # Always update ACTUAL position (for monitoring only, not for commands)
                    self.right_actual_pos = msg.position[i]
        
        # Check initialization
        if not self.initialized:
            if self.left_target_pos is not None and self.right_target_pos is not None:
                self.initialized = True
                self.get_logger().info('  ✓ Both grippers synced! Ready for keyboard control.')
            elif self.left_target_pos is not None or self.right_target_pos is not None:
                # Log available joints for debugging
                if not hasattr(self, '_logged_joints'):
                    self._logged_joints = True
                    self.get_logger().warn(f'  Available joints: {list(msg.name)}')
    
    def timer_callback(self):
        """Update TARGET positions based on key states and publish commands."""
        # Don't publish until we've synced with actual positions
        if not self.initialized:
            return
        
        dt = 1.0 / self.publish_rate
        delta = self.gripper_speed * dt
        
        # Left gripper - modify TARGET position only
        if self.keys_pressed['q']:  # Open (decrease position)
            self.left_target_pos = max(self.min_gripper, self.left_target_pos - delta)
        if self.keys_pressed['w']:  # Close (increase position)
            self.left_target_pos = min(self.max_gripper, self.left_target_pos + delta)
        
        # Right gripper - modify TARGET position only
        if self.keys_pressed['o']:  # Open (decrease position)
            self.right_target_pos = max(self.min_gripper, self.right_target_pos - delta)
        if self.keys_pressed['p']:  # Close (increase position)
            self.right_target_pos = min(self.max_gripper, self.right_target_pos + delta)
        
        # Publish gripper commands (TARGET positions, not actual)
        left_msg = Float64MultiArray()
        left_msg.data = [self.left_target_pos]
        self.left_gripper_pub.publish(left_msg)
        
        right_msg = Float64MultiArray()
        right_msg.data = [self.right_target_pos]
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
            if select.select([sys.stdin], [], [], 0.01)[0]:
                key = sys.stdin.read(1)
                
                if key == '\x1b':  # ESC
                    break
                elif key == '\x03':  # Ctrl+C
                    break
                elif key.lower() in node.keys_pressed:
                    # Set key pressed - will be processed in timer_callback
                    node.handle_key(key.lower(), True)
            
            # Process ROS callbacks
            rclpy.spin_once(node, timeout_sec=0.005)
            
            # Reset keys after ROS spin (ensures timer_callback sees the key press)
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
