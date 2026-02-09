"""
SmolVLA Inference Launch File

Launches the SmolVLA real-time inference node for OpenArm bimanual robot.

Usage:
    # Basic usage (dry-run mode first!)
    ros2 launch openarm_static_bimanual_bringup smolvla_inference.launch.py \
        policy_path:=/path/to/smolvla_openarm/checkpoints/020000/pretrained_model \
        task_description:="pick up the red block" \
        enable_control:=false

    # Production usage
    ros2 launch openarm_static_bimanual_bringup smolvla_inference.launch.py \
        policy_path:=/path/to/checkpoint \
        task_description:="pick up the object" \
        enable_control:=true

Author: OpenArm VLA Project
Date: 2026-02-05
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for SmolVLA inference."""
    
    # Declare launch arguments
    policy_path_arg = DeclareLaunchArgument(
        'policy_path',
        default_value='',
        description='Path to the trained SmolVLA checkpoint (required). '
                    'Example: /path/to/smolvla_openarm/checkpoints/020000/pretrained_model'
    )
    
    task_description_arg = DeclareLaunchArgument(
        'task_description',
        default_value='pick up the object',
        description='Task description for the VLA policy (natural language)'
    )
    
    device_arg = DeclareLaunchArgument(
        'device',
        default_value='cuda',
        description='Device to run inference on (cuda/cpu)'
    )
    
    inference_rate_arg = DeclareLaunchArgument(
        'inference_rate',
        default_value='10.0',
        description='Inference frequency in Hz (default: 10.0, max recommended: 20.0)'
    )
    
    enable_control_arg = DeclareLaunchArgument(
        'enable_control',
        default_value='false',  # Default to false for safety
        description='Whether to actually publish control commands. '
                    'Set to false for dry-run testing first!'
    )
    
    safety_velocity_limit_arg = DeclareLaunchArgument(
        'safety_velocity_limit',
        default_value='0.5',
        description='Maximum joint velocity in rad/s for safety limiting'
    )
    
    control_arm_arg = DeclareLaunchArgument(
        'control_arm',
        default_value='both',  # Default to both for 16-dim bimanual control
        description='Which arm to control: left, right, or both (16-dim model controls both)'
    )
    
    # Log startup info
    startup_info = LogInfo(
        msg=['Starting SmolVLA Inference with policy: ', LaunchConfiguration('policy_path')]
    )
    
    control_warning = LogInfo(
        msg=['⚠️ Control enabled: ', LaunchConfiguration('enable_control'), 
             ' - Set enable_control:=true only after dry-run testing!']
    )
    
    # SmolVLA inference node
    inference_node = Node(
        package='openarm_static_bimanual_bringup',
        executable='smolvla_inference_node.py',
        name='smolvla_inference',
        output='screen',
        parameters=[{
            'policy_path': LaunchConfiguration('policy_path'),
            'task_description': LaunchConfiguration('task_description'),
            'device': LaunchConfiguration('device'),
            'inference_rate': LaunchConfiguration('inference_rate'),
            'enable_control': LaunchConfiguration('enable_control'),
            'safety_velocity_limit': LaunchConfiguration('safety_velocity_limit'),
            'control_arm': LaunchConfiguration('control_arm'),
        }],
    )
    
    return LaunchDescription([
        # Arguments
        policy_path_arg,
        task_description_arg,
        device_arg,
        inference_rate_arg,
        enable_control_arg,
        safety_velocity_limit_arg,
        control_arm_arg,
        
        # Info
        startup_info,
        control_warning,
        
        # Node
        inference_node,
    ])
