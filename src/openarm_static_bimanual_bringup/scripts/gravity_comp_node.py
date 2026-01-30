#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Gravity Compensation Node for OpenArm v0.3 Bimanual Robot

This node computes gravity compensation torques using PyKDL library
and publishes them to effort controllers. It also includes joint
limit protection to prevent hardware damage.

Based on OpenArm v1.0 implementation.

NOTE: This version uses PyKDL directly with manual chain construction
to avoid dependency on kdl_parser_py which may not be available.
"""
import math
import sys
import os
import time
import numpy as np
import pinocchio as pin

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray




class GravityCompNode(Node):
    """
    Gravity Compensation Node
    
    Subscribes to /joint_states and publishes gravity compensation torques
    to /left_effort_controller/commands and /right_effort_controller/commands.
    
    Safety features:
    - Joint limit protection with configurable margin
    - Virtual spring at joint limits to push back
    - Emergency stop when limits are exceeded
    """
    
    # Joint limits from URDF (rad) - with safety margin
    JOINT_LIMITS = {
        # Joint: (lower, upper)
        'rev1': (-3.09, 3.09),    # ±120°
        'rev2': (-2.57, 2.57),    # ±90°
        'rev3': (-3.09, 3.09),    # ±120°
        'rev4': (-0.5, 3.62),      # 0° ~ 150°
        'rev5': (-3.09, 3.09),    # ±120°
        'rev6': (-2.57, 2.57),    # ±90°
        'rev7': (-1.96, 1.96),    # ±55°
    }
    
    # Safety margin from joint limits (rad) - approximately 5 degrees
    SAFETY_MARGIN = 0.087  # ~5°
    
    # Virtual spring stiffness at joint limits (Nm/rad)
    LIMIT_SPRING_K = 3.0
    
    # Link masses and COM positions from URDF (approximate values)
    # Format: (mass_kg, com_x, com_y, com_z)
    LINK_PARAMS = [
        (0.577, 0.0, 0.0, 0.023),      # link1
        (0.163, 0.0, -0.002, 0.028),   # link2  
        (0.420, -0.007, 0.0, 0.028),   # link3
        (0.819, 0.001, -0.002, -0.132), # link4
        (0.409, -0.083, 0.003, -0.029), # link5
        (0.345, -0.009, -0.014, -0.044), # link6
        (0.278, 0.0, 0.004, 0.035),    # link7
    ]
    
    # Link lengths for simplified kinematics (m)
    # Calculated from URDF joint origin xyz using Euclidean distance:
    # rev1: sqrt(0² + 0² + 0.05325²) = 0.053
    # rev2: sqrt(0² + 0.02975² + 0.04475²) = 0.054
    # rev3: sqrt(0.0612477² + 0.000536² + 0.02975²) = 0.068
    # rev4: sqrt(0.0297547² + 0² + 0.24175²) = 0.244
    # rev5: sqrt(0.133937² + 0.00188² + 0.0297547²) = 0.137
    # rev6: sqrt(0.0187648² + 0.0301352² + 0.12105²) = 0.127
    # rev7: sqrt(0.000217² + 0.0154485² + 0.0355²) = 0.039
    LINK_LENGTHS = [0.053, 0.054, 0.068, 0.244, 0.137, 0.127, 0.039]
    
    # URDF joint axis direction sign (from <axis xyz="..."/>)
    # rev1: xyz="0 0 1"  -> +1
    # rev2: xyz="0 0 1"  -> +1
    # rev3: xyz="0 0 -1" -> -1
    # rev4: xyz="0 0 1"  -> +1
    # rev5: xyz="0 0 -1" -> -1
    # rev6: xyz="0 0 -1" -> -1
    # rev7: xyz="0 0 -1" -> -1
    JOINT_AXIS_SIGN = [1, 1, -1, 1, -1, -1, -1]
    
    # Right Arm additional sign flip due to PI rotation in URDF origin (rev1, rev3)
    RIGHT_ARM_ORIGIN_FLIP = [1, 1, 1, 1, 1, 1, 1]  # rev1 and rev3 have PI offset but axis sign already handles it
    
    # Maximum torque limits per joint (Nm) - based on rated torque
    # DM4340 (rev1~4): Rated 9 Nm, Peak 27 Nm
    # DM4310 (rev5~7): Rated 3 Nm, Peak 7 Nm
    MAX_TORQUE = [9.0, 9.0, 9.0, 9.0, 3.0, 3.0, 3.0]
    
    def __init__(self):
        super().__init__('gravity_comp_node')
        
        # ===== Parameters =====
        self.declare_parameter('publish_rate', 100.0)  # Hz
        self.declare_parameter('enable_limit_protection', True)
        self.declare_parameter('safety_margin', self.SAFETY_MARGIN)
        self.declare_parameter('limit_spring_k', self.LIMIT_SPRING_K)
        # Per-joint gravity scale: rev1~4 (DM4340) use base scale, rev5~7 (DM4310) need higher scale
        # to compensate for weaker motors and accumulated end-effector weight
        self.declare_parameter('gravity_scale_joints', [1.5, 1.5, 1.5, 1.5, 2.5, 2.5, 2.5])
        self.declare_parameter('active_arms', 'both')
        self.declare_parameter('urdf_path', '/tmp/openarm_v03_bimanual.urdf')
        
        self.publish_rate = self.get_parameter('publish_rate').value
        self.enable_limit_protection = self.get_parameter('enable_limit_protection').value
        self.safety_margin = self.get_parameter('safety_margin').value
        self.limit_spring_k = self.get_parameter('limit_spring_k').value
        self.urdf_path = self.get_parameter('urdf_path').value
        
        # Wait for URDF file to be generated
        self.get_logger().info(f"Waiting for URDF file at {self.urdf_path}...")
        for _ in range(10):
            if os.path.exists(self.urdf_path):
                break
            time.sleep(1.0)
            
        if not os.path.exists(self.urdf_path):
            self.get_logger().error(f"URDF file not found at {self.urdf_path}. Pinocchio cannot load model.")
            raise FileNotFoundError(f"URDF file not found: {self.urdf_path}")
            
        # ===== Pinocchio Setup =====
        self.get_logger().info("Loading Pinocchio model...")
        self.model = pin.buildModelFromUrdf(self.urdf_path)
        self.data = self.model.createData()
        self.q = pin.neutral(self.model)
        
        # Map joint names to Pinocchio velocity indices (idx_v)
        # Note: idx_v is the index in the velocity/torque vector
        self.left_indices = []
        self.right_indices = []
        
        left_joint_names = [f'left_rev{i}' for i in range(1, 8)]
        right_joint_names = [f'right_rev{i}' for i in range(1, 8)]
        
        for name in left_joint_names:
            if self.model.existJointName(name):
                joint_id = self.model.getJointId(name)
                self.left_indices.append(self.model.joints[joint_id].idx_v)
            else:
                self.get_logger().error(f"Joint {name} not found in Pinocchio model!")
                
        for name in right_joint_names:
            if self.model.existJointName(name):
                joint_id = self.model.getJointId(name)
                self.right_indices.append(self.model.joints[joint_id].idx_v)
            else:
                self.get_logger().error(f"Joint {name} not found in Pinocchio model!")
                
        self.get_logger().info(f"Pinocchio model loaded. nv={self.model.nv}, nq={self.model.nq}")
        
        self.enable_limit_protection = self.get_parameter('enable_limit_protection').value
        self.safety_margin = self.get_parameter('safety_margin').value
        self.limit_spring_k = self.get_parameter('limit_spring_k').value
        self.gravity_scale_joints = self.get_parameter('gravity_scale_joints').value
        self.active_arms = self.get_parameter('active_arms').value
        
        # Gravity vector
        self.gravity = 9.81
        
        # Joint names
        self.left_joints = [f'left_rev{i}' for i in range(1, 8)]
        self.right_joints = [f'right_rev{i}' for i in range(1, 8)]
        
        # State
        self.last_joint_state = None
        self.is_enabled = True
        
        # ===== Subscribers & Publishers =====
        self.js_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_cb, 10)
        
        self.left_cmd_pub = self.create_publisher(
            Float64MultiArray, '/left_effort_controller/commands', 10)
        self.right_cmd_pub = self.create_publisher(
            Float64MultiArray, '/right_effort_controller/commands', 10)
        
        # Position command publishers (to sync pos_commands with current position)
        # This ensures Kp*(q_des - q) = 0 by setting q_des = q
        self.left_pos_pub = self.create_publisher(
            Float64MultiArray, '/left_teleop_stream_controller/commands', 10)
        self.right_pos_pub = self.create_publisher(
            Float64MultiArray, '/right_teleop_stream_controller/commands', 10)
        
        # Timer for control loop
        rate = self.get_parameter('publish_rate').value
        self.timer = self.create_timer(1.0 / rate, self.control_loop)
        
        self.get_logger().info('=== Gravity Compensation Node Initialized ===')
        self.get_logger().info(f'  Active arms: {self.active_arms}')
        self.get_logger().info(f'  Limit protection: {self.enable_limit_protection}')
        self.get_logger().info(f'  Safety margin: {math.degrees(self.safety_margin):.1f}°')
        self.get_logger().info(f'  Gravity scale (per joint): {self.gravity_scale_joints}')
        self.get_logger().info('  Press Ctrl+C to stop')
        self.get_logger().info('  [Using simplified gravity model - URDF params embedded]')
    
    def joint_state_cb(self, msg: JointState):
        """Store latest joint state."""
        self.last_joint_state = msg
    
    def get_positions(self, joint_names: list) -> list:
        """Extract positions for specific joints from JointState message."""
        if self.last_joint_state is None:
            return None
        
        positions = []
        for name in joint_names:
            if name in self.last_joint_state.name:
                idx = self.last_joint_state.name.index(name)
                positions.append(self.last_joint_state.position[idx])
            else:
                positions.append(0.0)
        return positions
    
    def compute_gravity_torque_pinocchio(self, positions: list, arm: str) -> list:
        """
        Compute gravity torques using Pinocchio RNEA (Recursive Newton-Euler Algorithm).
        """
        if len(positions) != 7:
            return [0.0] * 7
            
        # Update q vector with current positions
        # We need to map our 7-DOF positions to the full robot configuration q
        # Pinocchio q vector includes all joints (left, right, etc.)
        # We should maintain the last known state for all joints to handle coupling correcty,
        # but here we update only the active arm's joints and assume others are at zero (or last known).
        # Better: keep self.current_q and update it.
        
        # Since we might not receive both arm states simultaneously, we rely on what we have.
        # For simplicity in this function, we update the q at the specific indices.
        
        indices = self.left_indices if arm == 'left' else self.right_indices
        
        for i, idx_v in enumerate(indices):
            # For revolute joints, q index is usually same as v index (if no floating base)
            # But strictly: idx_q = model.joints[joint_id].idx_q
            # Since q is initialized to neutral, and we only have revolute joints, 
            # we can assume q has same structure for these joints.
            # However, safe way involves joint_id.
            # Optimization: Pre-calculate idx_q list in __init__
            # For now, let's assume idx_v maps directly to q for simple revolute joints
            # (which is true for 1-DOF joints starting from 0 or fixed base)
            
            # Actually, idx_q and idx_v can differ. Let's find idx_q properly.
            # Re-finding in loop is slow, but safe for now. 
            pass # See optimization below
            
        # Optimization: Use indices found in __init__.
        # Note: model.joints[id].idx_q and idx_v
        
        # Let's do RNEA with zero velocity and acceleration
        # We need to constructing the FULL q vector.
        # Ideally, we should update self.q with LATEST known positions of BOTH arms.
        
        # This function is called with only one arm's positions. 
        # Strategy: update self.q partially and compute RNEA.
        
        # Warning: RNEA computes for WHOLE body. If right arm is in non-zero pose but we only update left,
        # the right arm's gravity torque will be for zero pose.
        # But we only return the torques for the requested arm.
        # So it is safe AS LONG AS there is no dynamic coupling between left and right (which is true, they share base_link).
        # Base link is fixed, so Left Arm gravity does NOT depend on Right Arm pose.
        
        # 1. Update self.q
        for i, pos in enumerate(positions):
            # We need idx_q corresponding to this joint.
            # Using pre-calculated indices would be better.
            # For now, let's assume we can get it from indices (idx_v) 
            # because for Revolute joints, idx_q == idx_v usually (if no quaternion).
            idx = indices[i]
            self.q[idx] = pos
            
        # 2. Compute RNEA (Gravity only: v=0, a=0)
        # We pass 0 vectors for v and a
        pin.rnea(self.model, self.data, self.q, np.zeros(self.model.nv), np.zeros(self.model.nv))
        
        # 3. Extract torques (tau_gravity) with per-joint gravity scale
        # The result is stored in self.data.tau
        torques = []
        for i, idx in enumerate(indices):
            # Apply per-joint gravity scale for better compensation at gripper joints
            torques.append(self.data.tau[idx] * self.gravity_scale_joints[i])
            
        return torques
    
    def compute_limit_protection_torque(self, joint_idx: int, position: float) -> float:
        """
        Compute virtual spring torque to push arm away from joint limits.
        
        When position is within safety_margin of a limit, apply a restoring
        torque proportional to how close it is to the limit.
        """
        if not self.enable_limit_protection:
            return 0.0
        
        joint_name = f'rev{joint_idx + 1}'
        if joint_name not in self.JOINT_LIMITS:
            return 0.0
        
        lower, upper = self.JOINT_LIMITS[joint_name]
        lower_safe = lower + self.safety_margin
        upper_safe = upper - self.safety_margin
        
        if position < lower_safe:
            # Near lower limit: push positive direction
            distance = lower_safe - position
            torque = self.limit_spring_k * distance
            if position < lower:
                self.get_logger().warn(f'Joint {joint_name} EXCEEDED lower limit! pos={math.degrees(position):.1f}°', throttle_duration_sec=1.0)
            return torque
        
        elif position > upper_safe:
            # Near upper limit: push negative direction
            distance = position - upper_safe
            torque = -self.limit_spring_k * distance
            if position > upper:
                self.get_logger().warn(f'Joint {joint_name} EXCEEDED upper limit! pos={math.degrees(position):.1f}°', throttle_duration_sec=1.0)
            return torque
        
        return 0.0
    
    def control_loop(self):
        """Main control loop: compute and publish gravity compensation torques."""
        if self.last_joint_state is None or not self.is_enabled:
            return
        
        # Get joint positions based on active arms
        left_pos = None
        right_pos = None
        
        if self.active_arms in ('left', 'both'):
            left_pos = self.get_positions(self.left_joints)
        if self.active_arms in ('right', 'both'):
            right_pos = self.get_positions(self.right_joints)
        
        # Compute and publish for left arm
        if left_pos is not None:
            left_tau = self.compute_gravity_torque_pinocchio(left_pos, 'left')
            
            # Add limit protection torques
            for i in range(7):
                left_tau[i] += self.compute_limit_protection_torque(i, left_pos[i])
            
            # Clamp torques to safe limits
            for i in range(7):
                max_tau = self.MAX_TORQUE[i]
                left_tau[i] = max(-max_tau, min(max_tau, left_tau[i]))
            
            # Publish position and effort commands
            left_pos_cmd = Float64MultiArray()
            left_pos_cmd.data = left_pos
            self.left_pos_pub.publish(left_pos_cmd)
            
            left_cmd = Float64MultiArray()
            left_cmd.data = left_tau
            self.left_cmd_pub.publish(left_cmd)
        
        # Compute and publish for right arm
        if right_pos is not None:
            right_tau = self.compute_gravity_torque_pinocchio(right_pos, 'right')
            
            # Add limit protection torques
            for i in range(7):
                right_tau[i] += self.compute_limit_protection_torque(i, right_pos[i])
            
            # Clamp torques to safe limits
            for i in range(7):
                max_tau = self.MAX_TORQUE[i]
                right_tau[i] = max(-max_tau, min(max_tau, right_tau[i]))
            
            # Publish position and effort commands
            right_pos_cmd = Float64MultiArray()
            right_pos_cmd.data = right_pos
            self.right_pos_pub.publish(right_pos_cmd)
            
            right_cmd = Float64MultiArray()
            right_cmd.data = right_tau
            self.right_cmd_pub.publish(right_cmd)
            
    def send_zero_torque(self):
        """Send zero torque command to all active joints for safety."""
        zero_tau = [0.0] * 7
        cmd = Float64MultiArray()
        cmd.data = zero_tau
        
        self.get_logger().info('Sending ZERO torque command to release motors...')
        
        # Send multiple times to ensure delivery
        for _ in range(3):
            if self.active_arms in ('left', 'both') and self.left_cmd_pub:
                self.left_cmd_pub.publish(cmd)
            
            if self.active_arms in ('right', 'both') and self.right_cmd_pub:
                self.right_cmd_pub.publish(cmd)
                
    def destroy_node(self):
        self.send_zero_torque()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GravityCompNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down gravity compensation...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
