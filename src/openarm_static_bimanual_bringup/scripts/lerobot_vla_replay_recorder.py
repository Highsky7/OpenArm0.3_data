#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
LeRobot VLA Replay Recorder for OpenArm Bimanual Robot (Phase 2)

Replays trajectory from Phase 1 dataset while recording camera observations.
This creates complete VLA datasets with observation + action data.

Features:
- Load trajectory dataset from Phase 1
- Replay with gravity compensation mode
- Record camera observations during replay
- Generate complete VLA dataset for model training

Usage:
    ros2 run openarm_static_bimanual_bringup lerobot_vla_replay_recorder.py \\
        --ros-args \\
        -p trajectory_dataset:=~/lerobot_datasets/openarm_trajectory \\
        -p vla_dataset:=~/lerobot_datasets/openarm_vla

Part of 2-Phase VLA Data Collection Workflow:
    Phase 1: Manual teaching -> trajectory_dataset (lerobot_trajectory_recorder.py)
    Phase 2: Replay + record -> vla_dataset (this script)
"""
import json
import os
import time
import threading
from pathlib import Path
from threading import Lock
from typing import Optional

import cv2
import numpy as np
import pandas as pd
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import CompressedImage, JointState
from std_msgs.msg import Float64MultiArray

# -----------------------------------------------------------------------------
# MONKEY PATCH: FORCE H.264 CODEC (MUST BE APPLIED BEFORE ANY LEROBOT IMPORTS)
# -----------------------------------------------------------------------------
# The installed `lerobot` library defaults to `libsvtav1` (AV1), which causes 
# codec incompatibility issues when re-opening datasets (e.g., episode 2+).
# We patch `video_utils.encode_video_frames` globally to enforce H.264.
# -----------------------------------------------------------------------------
import functools

try:
    # 1. Import video_utils directly (source of the function)
    import lerobot.datasets.video_utils
    from lerobot.datasets.video_utils import encode_video_frames

    print("🔧 [Monkey Patch] Enforcing 'h264' + 'yuv420p' globally...")

    # 2. Create the fixed partial function
    fixed_encode_video_frames = functools.partial(
        encode_video_frames,
        vcodec="h264",
        pix_fmt="yuv420p",
        overwrite=True
    )

    # 3. Patch the SOURCE module immediately
    lerobot.datasets.video_utils.encode_video_frames = fixed_encode_video_frames
    
    # 4. Patch LeRobotDataset module (in case it's already imported or imports differently)
    import lerobot.datasets.lerobot_dataset
    lerobot.datasets.lerobot_dataset.encode_video_frames = fixed_encode_video_frames

    print("✅ [Monkey Patch] Successfully patched `video_utils` and `lerobot_dataset`.")

except ImportError as e:
    print(f"⚠️ [Monkey Patch] Failed to apply patch: {e}")
    # Proceed anyway, hoping for the best (or let it fail if critical)

# LeRobot imports (Now they will use the patched VIDEO_UTILS)

from lerobot.datasets.lerobot_dataset import LeRobotDataset




class LeRobotVLAReplayRecorder(Node):
    """
    LeRobot VLA Replay Recorder - Phase 2 of 2-Phase VLA Collection.
    
    Replays a trajectory dataset while recording camera observations
    to create a complete VLA dataset for model training.
    """
    
    # Joint ordering
    JOINT_NAMES = [
        'left_rev1', 'left_rev2', 'left_rev3', 'left_rev4',
        'left_rev5', 'left_rev6', 'left_rev7', 'left_rev8',
        'right_rev1', 'right_rev2', 'right_rev3', 'right_rev4',
        'right_rev5', 'right_rev6', 'right_rev7', 'right_rev8',
    ]
    NUM_JOINTS = 16
    
    # Arm joints (excluding gripper)
    LEFT_ARM_JOINTS = ['left_rev1', 'left_rev2', 'left_rev3', 'left_rev4',
                       'left_rev5', 'left_rev6', 'left_rev7']
    RIGHT_ARM_JOINTS = ['right_rev1', 'right_rev2', 'right_rev3', 'right_rev4',
                        'right_rev5', 'right_rev6', 'right_rev7']
    
    # Camera configuration (using compressed image topics for better bandwidth)
    CAMERA_TOPICS = {
        'top': '/camera/cam_1/color/image_raw/compressed',
        'wrist_left': '/camera/cam_2/color/image_raw/compressed',
        'wrist_right': '/camera/cam_3/color/image_raw/compressed',
    }
    IMAGE_SIZE = (256, 256)
    
    def __init__(self):
        super().__init__('lerobot_vla_replay_recorder')
        
        # Parameters
        self.declare_parameter('trajectory_dataset', '')
        self.declare_parameter('vla_dataset', '')
        self.declare_parameter('episode_index', -1)  # -1 for all episodes
        self.declare_parameter('playback_speed', 1.0)
        self.declare_parameter('record_rate', 30.0)  # Hz
        self.declare_parameter('robot_type', 'openarm_static_bimanual')
        self.declare_parameter('task_description', 'bimanual manipulation task')
        self.declare_parameter('resume', True)
        self.declare_parameter('repeat_count', 1)  # Number of times to repeat trajectory for VLA episodes
        
        trajectory_path = self.get_parameter('trajectory_dataset').value
        vla_path = self.get_parameter('vla_dataset').value
        self.episode_index = self.get_parameter('episode_index').value
        self.playback_speed = self.get_parameter('playback_speed').value
        self.record_rate = self.get_parameter('record_rate').value
        self.robot_type = self.get_parameter('robot_type').value
        self.task_description = self.get_parameter('task_description').value
        self.resume = self.get_parameter('resume').value
        self.repeat_count = self.get_parameter('repeat_count').value
        
        if not trajectory_path:
            self.get_logger().error('No trajectory_dataset specified!')
            return
        
        if not vla_path:
            # Default: trajectory path + "_vla"
            vla_path = trajectory_path.rstrip('/') + '_vla'
        
        self.trajectory_path = Path(os.path.expanduser(trajectory_path))
        self.vla_path = Path(os.path.expanduser(vla_path))
        self.vla_repo_id = f"local/{self.vla_path.name}"
        
        # Original trajectory fps (will be loaded from info.json)
        self.trajectory_fps: float = 30.0  # Default fallback
        
        # State
        self.last_joint_state: Optional[JointState] = None
        self.cv_bridge = CvBridge()
        self.latest_images: dict[str, Optional[np.ndarray]] = {
            key: None for key in self.CAMERA_TOPICS.keys()
        }
        self.image_lock = Lock()
        self.joint_state_lock = Lock()
        
        # Callback groups for parallel processing
        # Images and joint states need separate callback groups to run in parallel
        self.image_callback_group = ReentrantCallbackGroup()
        self.joint_callback_group = MutuallyExclusiveCallbackGroup()
        self.timer_callback_group = MutuallyExclusiveCallbackGroup()
        
        # VLA Dataset
        self.vla_dataset: Optional[LeRobotDataset] = None
        
        # Publishers for gravity compensation mode
        self.left_gravity_comp_pub = self.create_publisher(
            Float64MultiArray,
            '/gravity_comp/left_external_position_cmd',
            10
        )
        self.right_gravity_comp_pub = self.create_publisher(
            Float64MultiArray,
            '/gravity_comp/right_external_position_cmd',
            10
        )
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
        
        # Subscriber: Joint states (separate callback group)
        self.js_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10,
            callback_group=self.joint_callback_group)
        
        # Subscribers: Cameras (compressed images) - ReentrantCallbackGroup allows parallel execution
        for key, topic in self.CAMERA_TOPICS.items():
            self.create_subscription(
                CompressedImage,
                topic,
                lambda msg, k=key: self.image_callback(msg, k),
                10,
                callback_group=self.image_callback_group
            )
            self.get_logger().info(f'Subscribed: {topic} -> observation.images.{key}')
        
        # Initialize VLA dataset
        self._init_vla_dataset()
        
        # Start replay+record
        self.get_logger().info('Waiting for cameras and joint states...')
        self.create_timer(2.0, self.start_replay_once, callback_group=self.timer_callback_group)
        self._started = False
    
    def _get_features(self) -> dict:
        """Define VLA dataset features with cameras."""
        features = {
            "observation.state": {
                "dtype": "float32",
                "shape": (self.NUM_JOINTS,),
                "names": self.JOINT_NAMES,
            },
            "action": {
                "dtype": "float32",
                "shape": (self.NUM_JOINTS,),
                "names": self.JOINT_NAMES,
            },
        }
        
        # Add camera features
        for key in self.CAMERA_TOPICS.keys():
            features[f"observation.images.{key}"] = {
                "dtype": "video",
                "shape": (self.IMAGE_SIZE[1], self.IMAGE_SIZE[0], 3),
                "names": ["height", "width", "channels"],
            }
        
        return features
    
    def _init_vla_dataset(self, force_create: bool = False):
        """
        Initialize VLA dataset.
        
        Args:
            force_create: If True, skip resume check (used for re-initialization)
        """
        try:
            # Close existing dataset if any (important for video encoder reset)
            if self.vla_dataset is not None:
                try:
                    # Don't call finalize() here - just release resources
                    del self.vla_dataset
                    self.vla_dataset = None
                except Exception:
                    pass
            
            if not force_create and self.resume and self.vla_path.exists():
                self.vla_dataset = LeRobotDataset(
                    self.vla_repo_id,
                    root=self.vla_path,
                )
                self.get_logger().info(
                    f'📂 Resumed VLA dataset: {self.vla_dataset.num_episodes} episodes')
            else:
                features = self._get_features()
                self.vla_dataset = LeRobotDataset.create(
                    repo_id=self.vla_repo_id,
                    fps=int(self.record_rate),
                    root=self.vla_path,
                    robot_type=self.robot_type,
                    features=features,
                    use_videos=True,
                    vcodec='h264',  # Use H.264 for better compatibility (AV1 has decoding issues)
                )
                self.get_logger().info(f'📁 Created new VLA dataset: {self.vla_path}')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize VLA dataset: {e}')
            raise
    
    def _reopen_dataset_for_next_episode(self):
        """
        Reopen dataset to reset video encoder state.
        
        This is crucial for H.264 encoding: each episode needs fresh encoder
        to ensure proper keyframe (I-frame) generation at the start.
        """
        self.get_logger().info('  🔄 Reopening dataset for clean video encoder state...')
        try:
            # Save current episode first
            if self.vla_dataset is not None:
                self.vla_dataset.save_episode()
                
                # Get current episode count before closing
                num_episodes = self.vla_dataset.num_episodes
                
                # Close and reopen dataset
                # Explicitly finalize to flush writers and release resources
                self.vla_dataset.finalize()
                del self.vla_dataset
                self.vla_dataset = None
                
                # Small delay to ensure file handles are released
                time.sleep(0.5)  # Increased delay slightly
                
                # Reopen in resume mode
                self.vla_dataset = LeRobotDataset(
                    self.vla_repo_id,
                    root=self.vla_path,
                )
                self.get_logger().info(f'  ✅ Dataset reopened: {self.vla_dataset.num_episodes} episodes')
        except Exception as e:
            self.get_logger().error(f'Failed to reopen dataset: {e}')
            raise
    
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
                self.get_logger().warn(f'Failed to decode compressed image ({image_key})')
                return
            
            # Convert BGR to RGB (OpenCV decodes as BGR)
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            
            resized = cv2.resize(cv_image, self.IMAGE_SIZE, interpolation=cv2.INTER_AREA)
            with self.image_lock:
                self.latest_images[image_key] = resized
                # Track when each camera was last updated
                if not hasattr(self, '_image_update_times'):
                    self._image_update_times = {}
                self._image_update_times[image_key] = time.monotonic()
        except Exception as e:
            self.get_logger().warn(f'Compressed image conversion error ({image_key}): {e}')
    
    def _get_current_images(self) -> dict[str, Optional[np.ndarray]]:
        """Get current images with thread safety."""
        with self.image_lock:
            return {k: v.copy() if v is not None else None 
                    for k, v in self.latest_images.items()}
    
    def _get_joint_positions(self) -> Optional[np.ndarray]:
        """Extract joint positions in correct order with thread safety."""
        with self.joint_state_lock:
            if self.last_joint_state is None:
                return None
            
            pos_dict = {}
            for i, name in enumerate(self.last_joint_state.name):
                if i < len(self.last_joint_state.position):
                    pos_dict[name] = self.last_joint_state.position[i]
            
            positions = []
            for joint_name in self.JOINT_NAMES:
                if joint_name in pos_dict:
                    positions.append(pos_dict[joint_name])
                else:
                    positions.append(0.0)
            
            return np.array(positions, dtype=np.float32)
    
    def _publish_arm_command(self, publisher, positions):
        """Publish arm position command."""
        msg = Float64MultiArray()
        # Explicitly convert to Python float to avoid numpy dtype issues
        msg.data = [float(p) for p in positions]
        publisher.publish(msg)
    
    def _publish_gripper_command(self, publisher, position):
        """Publish gripper position command."""
        msg = Float64MultiArray()
        msg.data = [float(position)]
        publisher.publish(msg)
    
    def start_replay_once(self):
        """Start replay once cameras are ready."""
        if self._started:
            return
        
        # Check cameras
        current_images = self._get_current_images()
        missing = [k for k, v in current_images.items() if v is None]
        if missing:
            self.get_logger().info(f'Waiting for cameras: {missing}')
            return
        
        if self.last_joint_state is None:
            self.get_logger().info('Waiting for joint states...')
            return
        
        self._started = True
        self.replay_and_record()
    
    def replay_and_record(self):
        """Main replay and record loop."""
        # Load original trajectory fps from info.json
        info_path = self.trajectory_path / 'meta' / 'info.json'
        if info_path.exists():
            try:
                with open(info_path, 'r') as f:
                    info = json.load(f)
                self.trajectory_fps = float(info.get('fps', 30.0))
                self.get_logger().info(f'📁 Loaded trajectory info: fps={self.trajectory_fps}')
            except Exception as e:
                self.get_logger().warn(f'Failed to load info.json: {e}, using default fps=30')
                self.trajectory_fps = 30.0
        else:
            self.get_logger().warn(f'info.json not found at {info_path}, using default fps=30')
            self.trajectory_fps = 30.0
        
        # Load trajectory dataset (supports multi-chunk LeRobot v3.0 format)
        data_dir = self.trajectory_path / 'data'
        parquet_files = sorted(data_dir.glob('**/*.parquet'))
        
        if not parquet_files:
            self.get_logger().error(f'No parquet files in {data_dir}')
            return
        
        # Load and concatenate all parquet files for multi-chunk support
        dataframes = [pd.read_parquet(f) for f in parquet_files]
        df = pd.concat(dataframes, ignore_index=True)
        df = df.sort_values(['episode_index', 'frame_index']).reset_index(drop=True)
        
        self.get_logger().info(f'Loaded {len(parquet_files)} parquet file(s), {len(df)} total frames')
        
        # Get episodes to process
        if self.episode_index >= 0:
            episodes = [self.episode_index]
        else:
            episodes = sorted(df['episode_index'].unique())
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('  Phase 2: VLA Replay + Record')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'  Trajectory source: {self.trajectory_path}')
        self.get_logger().info(f'  VLA output: {self.vla_path}')
        self.get_logger().info(f'  Trajectory episodes: {len(episodes)}')
        self.get_logger().info(f'  Repeat count: {self.repeat_count}')
        self.get_logger().info(f'  Total VLA episodes to create: {len(episodes) * self.repeat_count}')
        self.get_logger().info(f'  Original trajectory fps: {self.trajectory_fps} Hz')
        self.get_logger().info(f'  Effective playback fps: {self.trajectory_fps * self.playback_speed} Hz')
        self.get_logger().info(f'  VLA record rate: {self.record_rate} Hz')
        self.get_logger().info('=' * 60)
        
        # Process each episode with repeat count
        vla_episode_num = 0
        total_vla_episodes = len(episodes) * self.repeat_count
        
        for repeat_idx in range(self.repeat_count):
            self.get_logger().info(f'\n🔁 Repeat {repeat_idx + 1}/{self.repeat_count}')
            
            for ep_idx_num, ep_idx in enumerate(episodes):
                if not rclpy.ok():
                    break
                
                ep_df = df[df['episode_index'] == ep_idx].copy()
                ep_df = ep_df.sort_values('frame_index').reset_index(drop=True)
                
                vla_episode_num += 1
                self.get_logger().info(f'🎬 VLA Episode {vla_episode_num}/{total_vla_episodes} (trajectory ep {ep_idx}, repeat {repeat_idx + 1}): {len(ep_df)} frames')
                frames_recorded = self._process_episode(ep_df, ep_idx)
                
                # After each episode: reopen dataset to reset video encoder
                # This ensures each episode starts with a fresh H.264 keyframe
                is_last = (repeat_idx == self.repeat_count - 1) and (ep_idx_num == len(episodes) - 1)
                
                if not is_last and frames_recorded > 0:
                    self._reopen_dataset_for_next_episode()
                    self.get_logger().info('  ⏳ Waiting 2s before next episode... (safe to Ctrl+C here)')
                    time.sleep(2.0)
                elif frames_recorded > 0:
                    # Last episode: just save without reopening
                    self.vla_dataset.save_episode()
                    self.get_logger().info(f'💾 Last episode saved: {frames_recorded} frames')
        
        # Finalize
        self.vla_dataset.finalize()
        self.get_logger().info('=' * 60)
        self.get_logger().info('✅ VLA dataset finalized!')
        self.get_logger().info(f'   Path: {self.vla_path}')
        self.get_logger().info(f'   Episodes: {self.vla_dataset.num_episodes}')
        self.get_logger().info(f'   Total frames: {self.vla_dataset.num_frames}')
        self.get_logger().info('=' * 60)
    
    def _wait_for_fresh_images(self, timeout: float = 2.0) -> bool:
        """Wait for fresh images from all cameras before starting episode."""
        self.get_logger().info('  Waiting for fresh camera images...')
        
        # Clear current images to force waiting for new ones
        with self.image_lock:
            for key in self.latest_images:
                self.latest_images[key] = None
            if hasattr(self, '_image_update_times'):
                self._image_update_times.clear()
        
        start_wait = time.monotonic()
        while time.monotonic() - start_wait < timeout:
            # Let callbacks run
            time.sleep(0.05)
            
            # Check if all images are available
            current_images = self._get_current_images()
            if all(img is not None for img in current_images.values()):
                self.get_logger().info('  ✅ All cameras ready!')
                return True
        
        # Check which cameras are missing
        current_images = self._get_current_images()
        missing = [k for k, v in current_images.items() if v is None]
        self.get_logger().warn(f'  ⚠️ Timeout waiting for cameras: {missing}')
        return False
    
    def _move_to_episode_start(self, ep_df, duration: float = 2.0):
        """
        Smoothly move robot to the first frame position of the episode.
        
        This prevents sudden jumps between episodes when their start positions differ.
        Uses cubic interpolation for smooth motion.
        """
        # Get first frame's position
        first_row = ep_df.iloc[0]
        if 'observation.state' in first_row and first_row['observation.state'] is not None:
            target_positions = np.array(first_row['observation.state'])
        else:
            target_positions = np.array(first_row['action'])
        
        # Get current position
        current_positions = self._get_joint_positions()
        if current_positions is None:
            self.get_logger().warn('  Cannot get current position, skipping smooth move')
            return
        
        # Check if we're already close enough (within 5 degrees = 0.087 rad)
        max_diff = np.max(np.abs(target_positions - current_positions))
        if max_diff < 0.087:
            self.get_logger().info('  Already at episode start position')
            return
        
        self.get_logger().info(f'  📍 Moving to episode start position ({duration}s)...')
        
        # Interpolation parameters
        rate = 50.0  # Hz
        dt = 1.0 / rate
        total_steps = int(duration * rate)
        
        start_positions = current_positions.copy()
        start_time = time.monotonic()
        
        for step in range(total_steps + 1):
            if not rclpy.ok():
                break
            
            # Cubic interpolation: s(t) = 3t² - 2t³
            t = step / total_steps
            s = 3 * t**2 - 2 * t**3
            
            # Interpolate
            interp_positions = start_positions + s * (target_positions - start_positions)
            
            # Publish arm commands
            self._publish_arm_command(self.left_gravity_comp_pub, interp_positions[:7])
            self._publish_arm_command(self.right_gravity_comp_pub, interp_positions[8:15])
            self._publish_gripper_command(self.left_gripper_pub, interp_positions[7])
            self._publish_gripper_command(self.right_gripper_pub, interp_positions[15])
            
            # Maintain timing
            target_time = start_time + step * dt
            wait_time = target_time - time.monotonic()
            if wait_time > 0:
                time.sleep(wait_time)
        
        self.get_logger().info('  ✅ Reached episode start position')
    
    def _process_episode(self, ep_df, episode_idx: int) -> int:
        """
        Process a single episode: replay + record with absolute timing.
        
        Returns:
            Number of frames recorded (0 if skipped)
        """
        # Wait for fresh images before starting each episode
        if not self._wait_for_fresh_images(timeout=3.0):
            self.get_logger().error(f'Skipping episode {episode_idx}: cameras not ready')
            return 0
        
        # Move to episode start position smoothly (prevents sudden jumps)
        self._move_to_episode_start(ep_df, duration=2.0)
        
        # Use ORIGINAL trajectory fps for replay timing (not VLA record_rate)
        # This ensures the robot moves at the same speed as during recording
        trajectory_period = 1.0 / self.trajectory_fps
        frames_recorded = 0
        total_frames = len(ep_df)
        
        # Use monotonic time for precise timing (prevents drift from sleep inaccuracy)
        start_time = time.monotonic()
        
        for frame_idx, (idx, row) in enumerate(ep_df.iterrows()):
            if not rclpy.ok():
                break
            
            # Calculate absolute target time for this frame using TRAJECTORY fps
            # playback_speed > 1.0 means faster playback
            target_time = start_time + (frame_idx * trajectory_period / self.playback_speed)
            
            # Get target positions from observation.state (actual recorded positions)
            # Using observation.state instead of action prevents accumulated drift
            # because it represents where the robot actually was during recording
            if 'observation.state' in row and row['observation.state'] is not None:
                target_positions = np.array(row['observation.state'])
            else:
                # Fallback to action if observation.state not available
                target_positions = np.array(row['action'])
            
            # Extract arm and gripper positions
            left_arm_pos = target_positions[:7]
            left_gripper_pos = target_positions[7]
            right_arm_pos = target_positions[8:15]
            right_gripper_pos = target_positions[15]
            
            # Publish commands to gravity compensation node
            self._publish_arm_command(self.left_gravity_comp_pub, left_arm_pos)
            self._publish_arm_command(self.right_gravity_comp_pub, right_arm_pos)
            self._publish_gripper_command(self.left_gripper_pub, left_gripper_pos)
            self._publish_gripper_command(self.right_gripper_pub, right_gripper_pos)
            
            # Wait until target time
            # MultiThreadedExecutor handles callbacks in background threads,
            # so we just need to wait for the right timing
            current_time = time.monotonic()
            wait_time = target_time - current_time
            if wait_time > 0:
                time.sleep(wait_time)
            
            # Record current state + cameras
            current_positions = self._get_joint_positions()
            current_images = self._get_current_images()
            
            if current_positions is None:
                continue
            
            if any(img is None for img in current_images.values()):
                self.get_logger().warn('Skipping frame: missing camera image')
                continue
            
            # Debug: Check if images are actually being updated
            if frame_idx % 50 == 0 and hasattr(self, '_image_update_times'):
                now = time.monotonic()
                for cam_key, update_time in self._image_update_times.items():
                    age_ms = (now - update_time) * 1000
                    if age_ms > 100:  # Image older than 100ms is stale
                        self.get_logger().warn(f'  ⚠️ {cam_key} image is {age_ms:.0f}ms old (stale!)')
                    else:
                        self.get_logger().info(f'  ✅ {cam_key} image is {age_ms:.0f}ms old (fresh)')
            
            # Build VLA frame
            # Use original action from trajectory for training (represents intended motion)
            original_action = np.array(row['action'])
            frame = {
                'observation.state': current_positions.copy(),
                'action': original_action.copy(),  # Keep original action for VLA training
                'task': self.task_description,
            }
            
            # Add camera images
            for key, img in current_images.items():
                frame[f'observation.images.{key}'] = img
            
            self.vla_dataset.add_frame(frame)
            frames_recorded += 1
            
            # Log progress every 100 frames
            if frame_idx % 100 == 0:
                elapsed = time.monotonic() - start_time
                expected = frame_idx * trajectory_period / self.playback_speed
                drift_ms = (elapsed - expected) * 1000
                self.get_logger().info(
                    f'  Frame {frame_idx}/{total_frames} | Timing drift: {drift_ms:+.1f}ms')
        
        # Save episode (Note: save_episode is now called in _reopen_dataset_for_next_episode
        # for all but the last episode. For the last episode, we save here.)
        if frames_recorded > 1:
            # Don't save here - let the caller handle it via _reopen_dataset_for_next_episode
            # or finalize() for the last episode
            self.get_logger().info(f'✅ Episode recorded: {frames_recorded} frames')
        else:
            self.get_logger().warn('Episode too short, skipping')
        
        return frames_recorded


def main(args=None):
    rclpy.init(args=args)
    node = LeRobotVLAReplayRecorder()
    
    # Use MultiThreadedExecutor to process callbacks in parallel
    # This allows image callbacks and joint_state callbacks to run simultaneously
    executor = MultiThreadedExecutor(num_threads=6)  # 3 cameras + joint_states + timer + margin
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        if node.vla_dataset:
            node.vla_dataset.finalize()
            node.get_logger().info('Dataset finalized on interrupt')
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
