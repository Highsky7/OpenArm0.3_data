#!/usr/bin/env python3
"""
SmolVLA Real-time Inference Node for OpenArm Bimanual Robot

Runs the trained SmolVLA policy for real-time robot control.
Subscribes to camera images and joint states, performs inference,
and publishes action commands to control the robot.

This version is adapted for:
- lerobot 0.4.3 (pip installed)
- Checkpoint with 16-dim state/action (bimanual: left 8 + right 8)
- 256x256 image input
- Camera keys: camera1, camera2, camera3
- Joint layout: left_rev1~7 (arm) + left_rev8 (gripper)
               right_rev1~7 (arm) + right_rev8 (gripper)

Author: OpenArm VLA Project
Date: 2026-02-05
"""
import threading
import time
from collections import deque
from typing import Optional

import cv2
import numpy as np
import torch

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import CompressedImage, JointState
from std_msgs.msg import String, Float64MultiArray


class SmolVLAInferenceNode(Node):
    """
    Real-time SmolVLA inference node with ROS2 integration.
    
    Subscribes to:
        - /camera/cam_1/color/image_raw/compressed (camera1 / front)
        - /camera/cam_2/color/image_raw/compressed (camera2 / wrist_left)  
        - /camera/cam_3/color/image_raw/compressed (camera3 / wrist_right)
        - /joint_states (robot joint positions)
        - /vla/task_description (text task description)
    
    Publishes to:
        - /gravity_comp/left_external_position_cmd (left arm positions)
        - /gravity_comp/right_external_position_cmd (right arm positions)
        - /left_gripper_controller/commands (left gripper)
        - /right_gripper_controller/commands (right gripper)
    """
    
    # Camera topic mapping 
    # IMPORTANT: Maps ROS topic to the key names expected by the trained model
    # The model expects: observation.images.camera1, camera2, camera3
    # But preprocessor will rename from top/wrist_left/wrist_right to camera1/2/3
    CAMERA_TOPICS = {
        'observation.images.top': '/camera/cam_1/color/image_raw/compressed',
        'observation.images.wrist_left': '/camera/cam_2/color/image_raw/compressed',
        'observation.images.wrist_right': '/camera/cam_3/color/image_raw/compressed',
    }
    
    # Joint names - for 16-DOF state (bimanual: left 8 + right 8)
    # Matching the training data: left_rev1~8, right_rev1~8
    # Where rev8 corresponds to the gripper on each arm
    STATE_JOINT_NAMES = [
        # Left arm (8 joints: 7 arm + 1 gripper)
        'left_rev1', 'left_rev2', 'left_rev3', 'left_rev4',
        'left_rev5', 'left_rev6', 'left_rev7', 'left_rev8',
        # Right arm (8 joints: 7 arm + 1 gripper)
        'right_rev1', 'right_rev2', 'right_rev3', 'right_rev4',
        'right_rev5', 'right_rev6', 'right_rev7', 'right_rev8',
    ]
    
    # Index mapping for action output
    # Action format: [left_rev1..8, right_rev1..8]
    LEFT_ARM_INDICES = list(range(0, 7))    # left_rev1~7 (arm joints)
    LEFT_GRIPPER_INDEX = 7                   # left_rev8 (gripper)
    RIGHT_ARM_INDICES = list(range(8, 15))  # right_rev1~7 (arm joints)
    RIGHT_GRIPPER_INDEX = 15                 # right_rev8 (gripper)
    
    # Image size for the policy (must match training: 256x256)
    IMAGE_SIZE = (256, 256)
    
    def __init__(self):
        super().__init__('smolvla_inference_node')
        
        # Declare parameters
        self.declare_parameter('policy_path', '')
        self.declare_parameter('device', 'cuda')
        self.declare_parameter('inference_rate', 10.0)  # Hz
        self.declare_parameter('task_description', 'pick up the object')
        self.declare_parameter('action_horizon', 1)
        self.declare_parameter('enable_control', True)
        self.declare_parameter('safety_velocity_limit', 0.5)  # rad/s
        self.declare_parameter('control_arm', 'left')  # Which arm to control: 'left', 'right', or 'both'
        
        # Get parameters
        self.policy_path = self.get_parameter('policy_path').value
        self.device = self.get_parameter('device').value
        self.inference_rate = self.get_parameter('inference_rate').value
        self.task_description = self.get_parameter('task_description').value
        self.action_horizon = self.get_parameter('action_horizon').value
        self.enable_control = self.get_parameter('enable_control').value
        self.safety_velocity_limit = self.get_parameter('safety_velocity_limit').value
        self.control_arm = self.get_parameter('control_arm').value
        
        if not self.policy_path:
            self.get_logger().error('❌ policy_path parameter is required!')
            raise ValueError('policy_path parameter is required')
        
        # Thread safety locks
        self.image_lock = threading.Lock()
        self.joint_state_lock = threading.Lock()
        self.action_lock = threading.Lock()
        
        # State storage
        self.latest_images: dict[str, Optional[np.ndarray]] = {k: None for k in self.CAMERA_TOPICS}
        self.last_joint_state: Optional[JointState] = None
        self.current_task: str = self.task_description
        
        # Action queue (for action chunking)
        self.action_queue: deque = deque(maxlen=100)
        self.last_action: Optional[np.ndarray] = None
        
        # Inference status
        self._running = False
        self._inference_count = 0
        self._last_inference_time = 0.0
        
        # Callback groups for parallel processing
        self.image_callback_group = ReentrantCallbackGroup()
        self.joint_callback_group = MutuallyExclusiveCallbackGroup()
        self.inference_callback_group = MutuallyExclusiveCallbackGroup()
        
        # Load the policy
        self._load_policy()
        
        # Setup ROS2 subscribers
        self._setup_subscribers()
        
        # Setup ROS2 publishers
        self._setup_publishers()
        
        # Inference timer
        self.create_timer(
            1.0 / self.inference_rate,
            self.inference_callback,
            callback_group=self.inference_callback_group
        )
        
        self.get_logger().info(f'✅ SmolVLA Inference Node initialized')
        self.get_logger().info(f'   Policy: {self.policy_path}')
        self.get_logger().info(f'   Device: {self.device}')
        self.get_logger().info(f'   Inference Rate: {self.inference_rate} Hz')
        self.get_logger().info(f'   Task: {self.task_description}')
        self.get_logger().info(f'   Control Enabled: {self.enable_control}')
        self.get_logger().info(f'   Control Arm: {self.control_arm}')
        self.get_logger().info(f'   Image Size: {self.IMAGE_SIZE}')
    
    def _load_policy(self):
        """Load SmolVLA policy from pretrained checkpoint."""
        self.get_logger().info(f'🔄 Loading policy from {self.policy_path}...')
        
        try:
            # Import lerobot (should be 0.4.3 from pip)
            from lerobot.policies.smolvla.modeling_smolvla import SmolVLAPolicy
            from lerobot.policies.factory import make_pre_post_processors
            
            # Load policy using from_pretrained
            self.policy = SmolVLAPolicy.from_pretrained(self.policy_path)
            self.policy.to(self.device)
            self.policy.eval()
            
            # Load preprocessor and postprocessor from checkpoint
            self.preprocessor, self.postprocessor = make_pre_post_processors(
                self.policy.config,
                pretrained_path=self.policy_path
            )
            
            # Reset policy state
            self.policy.reset()
            
            self.get_logger().info(f'✅ Policy loaded successfully!')
            self.get_logger().info(f'   Policy type: {self.policy.config.type}')
            
            # Log input/output features
            if hasattr(self.policy.config, 'input_features'):
                self.get_logger().info(f'   Input features: {list(self.policy.config.input_features.keys())}')
            if hasattr(self.policy.config, 'output_features'):
                self.get_logger().info(f'   Output features: {list(self.policy.config.output_features.keys())}')
            
        except Exception as e:
            self.get_logger().error(f'❌ Failed to load policy: {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())
            raise
    
    def _setup_subscribers(self):
        """Setup ROS2 subscribers for sensors."""
        # Subscribe to cameras (compressed images)
        for key, topic in self.CAMERA_TOPICS.items():
            self.create_subscription(
                CompressedImage,
                topic,
                lambda msg, k=key: self.image_callback(msg, k),
                10,
                callback_group=self.image_callback_group
            )
            self.get_logger().info(f'📷 Subscribed to {topic} -> {key}')
        
        # Subscribe to joint states
        self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10,
            callback_group=self.joint_callback_group
        )
        self.get_logger().info(f'🦾 Subscribed to /joint_states')
        
        # Subscribe to task description (dynamic task updates)
        self.create_subscription(
            String,
            '/vla/task_description',
            self.task_callback,
            10
        )
        self.get_logger().info(f'📝 Subscribed to /vla/task_description')
    
    def _setup_publishers(self):
        """Setup ROS2 publishers for robot control."""
        # Left arm position command
        self.left_arm_pub = self.create_publisher(
            Float64MultiArray,
            '/gravity_comp/left_external_position_cmd',
            10
        )
        
        # Right arm position command
        self.right_arm_pub = self.create_publisher(
            Float64MultiArray,
            '/gravity_comp/right_external_position_cmd',
            10
        )
        
        # Gripper commands
        self.left_gripper_pub = self.create_publisher(
            Float64MultiArray,
            '/left_gripper_controller/commands',
            10
        )
        
        self.right_gripper_pub = self.create_publisher(
            Float64MultiArray,
            '/right_gripper_controller/commands',
            10
        )
        
        self.get_logger().info(f'🤖 Publishers created for robot control')
    
    def joint_state_callback(self, msg: JointState):
        """Store latest joint state with thread safety."""
        with self.joint_state_lock:
            self.last_joint_state = msg
    
    def image_callback(self, msg: CompressedImage, image_key: str):
        """Store and resize latest compressed camera image."""
        try:
            # Decode compressed image (JPEG/PNG) to numpy array
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            if cv_image is None:
                self.get_logger().warn(f'Failed to decode image ({image_key})')
                return
            
            # Convert BGR to RGB (OpenCV decodes as BGR)
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            
            # Resize to policy input size (256x256 for this checkpoint)
            resized = cv2.resize(cv_image, self.IMAGE_SIZE, interpolation=cv2.INTER_AREA)
            
            with self.image_lock:
                self.latest_images[image_key] = resized
                
        except Exception as e:
            self.get_logger().warn(f'Image callback error ({image_key}): {e}')
    
    def task_callback(self, msg: String):
        """Update current task description."""
        self.current_task = msg.data
        self.get_logger().info(f'📝 Task updated: {self.current_task}')
        # Reset policy state when task changes
        self.policy.reset()
        with self.action_lock:
            self.action_queue.clear()
    
    def _get_current_images(self) -> dict[str, Optional[np.ndarray]]:
        """Get current images with thread safety."""
        with self.image_lock:
            return {k: v.copy() if v is not None else None 
                    for k, v in self.latest_images.items()}
    
    def _get_joint_positions(self) -> Optional[np.ndarray]:
        """Extract 16-dim joint positions (state) with thread safety."""
        with self.joint_state_lock:
            if self.last_joint_state is None:
                return None
            
            pos_dict = {}
            for i, name in enumerate(self.last_joint_state.name):
                if i < len(self.last_joint_state.position):
                    pos_dict[name] = self.last_joint_state.position[i]
            
            # Extract 16-dim state: [left_rev1~8, right_rev1~8]
            positions = []
            missing_joints = []
            for joint_name in self.STATE_JOINT_NAMES:
                if joint_name in pos_dict:
                    positions.append(pos_dict[joint_name])
                else:
                    missing_joints.append(joint_name)
                    positions.append(0.0)
            
            if missing_joints and not hasattr(self, '_logged_missing_joints'):
                self.get_logger().warn(f'Missing joints: {missing_joints}, using 0.0')
                self._logged_missing_joints = True
            
            return np.array(positions, dtype=np.float32)
    
    def _prepare_observation(self) -> Optional[dict]:
        """
        Prepare observation dictionary for policy inference.
        
        Returns None if sensors are not ready.
        """
        # Get current images
        images = self._get_current_images()
        missing_cameras = [k for k, v in images.items() if v is None]
        if missing_cameras:
            return None
        
        # Get joint positions (16-dim: left_rev1~8 + right_rev1~8)
        state = self._get_joint_positions()
        if state is None:
            return None
        
        # Build observation dict (matching the preprocessor expectation)
        # The preprocessor will rename these keys and handle normalization
        observation = {
            'observation.state': state,  # 16-dim state vector
            'task': self.current_task,   # Task description
        }
        
        # Add images (HWC format, uint8 -> will be converted by preprocessor)
        for key, img in images.items():
            observation[key] = img
        
        return observation
    
    def inference_callback(self):
        """Main inference loop - runs at inference_rate Hz."""
        if not self._running:
            # Check if all sensors are ready
            images = self._get_current_images()
            state = self._get_joint_positions()
            
            missing = [k for k, v in images.items() if v is None]
            if missing or state is None:
                if not hasattr(self, '_last_waiting_log') or time.time() - self._last_waiting_log > 2.0:
                    waiting_for = []
                    if missing:
                        waiting_for.append(f"cameras: {missing}")
                    if state is None:
                        waiting_for.append("joint states")
                    self.get_logger().info(f'⏳ Waiting for: {", ".join(waiting_for)}')
                    self._last_waiting_log = time.time()
                return
            
            self._running = True
            self.get_logger().info('🚀 All sensors ready! Starting inference...')
        
        # Check if we have queued actions to execute
        with self.action_lock:
            if self.action_queue:
                action = self.action_queue.popleft()
                self._execute_action(action)
                return
        
        # Prepare observation
        observation = self._prepare_observation()
        if observation is None:
            self.get_logger().warn('⚠️ Failed to prepare observation')
            return
        
        # Run inference
        try:
            start_time = time.time()
            
            # Preprocess observation (normalization, device transfer, etc.)
            batch = self.preprocessor(observation)
            
            with torch.no_grad():
                # Use select_action for single-step inference
                action = self.policy.select_action(batch)
            
            # Postprocess action (denormalization)
            action = self.postprocessor(action)
            
            inference_time = time.time() - start_time
            
            # Convert action tensor to numpy
            if isinstance(action, torch.Tensor):
                action_np = action.squeeze().cpu().numpy()
            else:
                action_np = np.array(action).squeeze()
            
            # Execute the action
            self._execute_action(action_np)
            
            # Log statistics periodically
            self._inference_count += 1
            if self._inference_count % 50 == 0:
                self.get_logger().info(
                    f'📊 Inference #{self._inference_count}: {inference_time*1000:.1f}ms, '
                    f'action: {action_np}'
                )
            
        except Exception as e:
            self.get_logger().error(f'❌ Inference error: {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())
    
    def _execute_action(self, action: np.ndarray):
        """
        Execute action by publishing to robot control topics.
        
        For the 16-dim action model, action is:
            [left_rev1..left_rev8, right_rev1..right_rev8]
        
        Where:
            - left_rev1~7: left arm joint positions (7 DOF)
            - left_rev8: left gripper position
            - right_rev1~7: right arm joint positions (7 DOF)
            - right_rev8: right gripper position
        """
        if not self.enable_control:
            return
        
        # Ensure action is 1D and has 16 elements
        action = action.flatten()
        if len(action) != 16:
            self.get_logger().error(
                f'❌ Expected 16-dim action, got {len(action)}-dim. '
                f'Check checkpoint config!'
            )
            return
        
        # Safety check: limit velocity if we have previous action
        if self.last_action is not None:
            dt = 1.0 / self.inference_rate
            velocity = (action - self.last_action) / dt
            max_vel = np.max(np.abs(velocity))
            if max_vel > self.safety_velocity_limit:
                self.get_logger().warn(
                    f'⚠️ Velocity limit exceeded: {max_vel:.2f} rad/s, scaling down'
                )
                scale = self.safety_velocity_limit / max_vel
                action = self.last_action + (action - self.last_action) * scale
        
        self.last_action = action.copy()
        
        # Extract arm and gripper commands
        # Left arm: indices 0-6 (7 joints), gripper: index 7
        # Right arm: indices 8-14 (7 joints), gripper: index 15
        left_arm_positions = action[self.LEFT_ARM_INDICES]    # 7 joints
        left_gripper_pos = action[self.LEFT_GRIPPER_INDEX]     # 1 gripper
        right_arm_positions = action[self.RIGHT_ARM_INDICES]  # 7 joints
        right_gripper_pos = action[self.RIGHT_GRIPPER_INDEX]   # 1 gripper
        
        # Publish based on control_arm setting
        if self.control_arm in ['left', 'both']:
            # Left arm positions (7 DOF)
            left_arm_msg = Float64MultiArray()
            left_arm_msg.data = [float(p) for p in left_arm_positions]
            self.left_arm_pub.publish(left_arm_msg)
            
            # Left gripper
            left_gripper_msg = Float64MultiArray()
            left_gripper_msg.data = [float(left_gripper_pos)]
            self.left_gripper_pub.publish(left_gripper_msg)
        
        if self.control_arm in ['right', 'both']:
            # Right arm positions (7 DOF)
            right_arm_msg = Float64MultiArray()
            right_arm_msg.data = [float(p) for p in right_arm_positions]
            self.right_arm_pub.publish(right_arm_msg)
            
            # Right gripper
            right_gripper_msg = Float64MultiArray()
            right_gripper_msg.data = [float(right_gripper_pos)]
            self.right_gripper_pub.publish(right_gripper_msg)


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    
    try:
        node = SmolVLAInferenceNode()
        
        # Use multi-threaded executor for parallel callbacks
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(node)
        
        try:
            executor.spin()
        except KeyboardInterrupt:
            node.get_logger().info('🛑 Shutting down...')
        finally:
            executor.shutdown()
            node.destroy_node()
            
    except Exception as e:
        print(f'❌ Failed to start node: {e}')
        import traceback
        traceback.print_exc()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
