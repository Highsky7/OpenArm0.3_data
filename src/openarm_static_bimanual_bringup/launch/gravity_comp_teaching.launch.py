#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Gravity Compensation Teaching Launch File

Launches the OpenArm bimanual robot in gravity compensation mode
for manual teaching / joint recording.

Usage:
    ros2 launch openarm_static_bimanual_bringup gravity_comp_teaching.launch.py
    ros2 launch openarm_static_bimanual_bringup gravity_comp_teaching.launch.py use_mock_hardware:=true
"""
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
    ExecuteProcess,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ===== Arguments =====
    declared_arguments = [
        DeclareLaunchArgument(
            'use_mock_hardware',
            default_value='false',
            description='Use mock hardware for simulation'
        ),
        DeclareLaunchArgument(
            'can_device',
            default_value='can0',
            description='CAN device name'
        ),
        DeclareLaunchArgument(
            'urdf_path',
            default_value='/tmp/openarm_v03_bimanual.urdf',
            description='Path to static URDF file for KDL'
        ),
        DeclareLaunchArgument(
            'enable_recorder',
            default_value='true',
            description='Launch the recorder node'
        ),
        DeclareLaunchArgument(
            'record_rate',
            default_value='50.0',
            description='Recording rate in Hz'
        ),
        DeclareLaunchArgument(
            'save_dir',
            default_value='~/openarm_official_ws',
            description='Directory to save recorded trajectories'
        ),
        DeclareLaunchArgument(
            'active_arms',
            default_value='both',
            description="Which arms to control: 'left', 'right', or 'both'"
        ),
        DeclareLaunchArgument(
            'use_grippers',
            default_value='true',
            description='Enable gripper joints in URDF for recording'
        ),
        # Arduino Gripper Bridge (run keyboard controller separately)
        DeclareLaunchArgument(
            'enable_gripper_bridge',
            default_value='false',
            description='Enable Arduino gripper bridge (run keyboard controller in separate terminal)'
        ),
        DeclareLaunchArgument(
            'servo_port',
            default_value='auto',
            description='Arduino serial port for gripper bridge'
        ),
    ]
    
    use_mock_hardware = LaunchConfiguration('use_mock_hardware')
    can_device = LaunchConfiguration('can_device')
    urdf_path = LaunchConfiguration('urdf_path')
    enable_recorder = LaunchConfiguration('enable_recorder')
    record_rate = LaunchConfiguration('record_rate')
    save_dir = LaunchConfiguration('save_dir')
    active_arms = LaunchConfiguration('active_arms')
    use_grippers = LaunchConfiguration('use_grippers')
    enable_gripper_bridge = LaunchConfiguration('enable_gripper_bridge')
    servo_port = LaunchConfiguration('servo_port')
    
    pkg_share = FindPackageShare('openarm_static_bimanual_bringup')
    description_pkg_share = FindPackageShare('openarm_static_bimanual_description')

    # ===== Generate URDF for Pinocchio =====
    # We need a physical file for Pinocchio to load.
    # Execute xacro and save to urdf_path.
    # Note: Using shell=True with a single command string to avoid spacing issues
    urdf_gen_process = ExecuteProcess(
        cmd=[
            'bash', '-c',
            [
                FindExecutable(name='xacro'), ' ',
                PathJoinSubstitution([description_pkg_share, 'urdf', 'openarm_static_bimanual.urdf.xacro']), ' ',
                'use_grippers:=', use_grippers, ' ',
                'use_mock_hardware:=', use_mock_hardware, ' ',
                'mount_half_x:=0.30 ',
                '-o ', urdf_path
            ]
        ],
        output='screen'
    )
    
    # ===== Include base bringup launch =====
    # This includes robot_state_publisher, controller_manager, etc.
    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_share, 'launch', 'sbopenarm.launch.py'])
        ),
        launch_arguments={
            'use_mock_hardware': use_mock_hardware,
            'can_device': can_device,
            'disable_torque': 'false',  # Enable torque for gravity comp
            'active_mode': 'teleop',  # Use teleop mode for position sync (Kp nullification)
            'rviz': 'true',
            'use_grippers': 'true',  # Enable gripper joints for recording
        }.items()
    )
    
    # ===== Effort controller spawners =====
    left_effort_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['left_effort_controller', '-c', '/controller_manager'],
        output='screen',
    )
    
    right_effort_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['right_effort_controller', '-c', '/controller_manager'],
        output='screen',
    )
    
    # ===== Gravity Compensation Node =====
    # Per-joint gravity scale tuning:
    #   rev1~4 (DM4340, 9Nm rated): base scale 1.5
    #   rev5~7 (DM4310, 3Nm rated): higher scale 2.5 to compensate for weaker motors
    # Adjust these values based on actual robot behavior:
    #   - If joint still sags: increase the scale
    #   - If joint resists manual movement: decrease the scale
    gravity_comp_node = Node(
        package='openarm_static_bimanual_bringup',
        executable='gravity_comp_node.py',
        name='gravity_comp_node',
        output='screen',
        parameters=[{
            'urdf_path': urdf_path,
            'left_base_link': 'left_dummy_link',
            'left_tip_link': 'left_link8',
            'right_base_link': 'right_dummy_link',
            'right_tip_link': 'right_link8',
            'publish_rate': 100.0,
            'enable_limit_protection': True,
            'safety_margin': 0.087,  # ~5 degrees
            'limit_spring_k': 5.0,
            'active_arms': active_arms,
            # Per-joint gravity scale: [rev1, rev2, rev3, rev4, rev5, rev6, rev7]
            'gravity_scale_joints': [1.5, 1.5, 1.5, 1.5, 1.8, 1.8, 1.8],
        }],
    )
    
    # ===== Continuous Recorder Node =====
    recorder_node = Node(
        package='openarm_static_bimanual_bringup',
        executable='continuous_recorder_node.py',
        name='continuous_recorder',
        output='screen',
        parameters=[{
            'record_rate': record_rate,
            'save_dir': save_dir,
            'file_format': 'json',
            'enable_limit_warning': True,
        }],
        condition=IfCondition(enable_recorder),
    )
    
    # ===== Delayed start for gravity comp and recorder =====
    # Wait for effort controllers to be ready
    # Note: teleop_stream_controller is already spawned by sbopenarm.launch.py (active_mode='teleop')
    delayed_nodes = TimerAction(
        period=5.0,  # Wait 5 seconds after launch
        actions=[
            gravity_comp_node,
            recorder_node,
        ]
    )
    
    # ===== Arduino Gripper Bridge (conditional) =====
    # NOTE: Keyboard controller should be run separately via ros2 run
    #       in a separate terminal for keyboard input to work
    gripper_bridge_node = Node(
        package='openarm_arduino_bridge',
        executable='bimanual_servo_bridge',
        name='gripper_bridge',
        output='screen',
        parameters=[{'port': servo_port, 'baud': 115200}],
        condition=IfCondition(enable_gripper_bridge),
    )
    
    delayed_gripper_nodes = TimerAction(
        period=6.0,
        actions=[gripper_bridge_node]
    )
    
    return LaunchDescription(
        declared_arguments + [
            urdf_gen_process,  # Generate URDF first
            base_launch,
            # Spawn effort controllers after base launch is ready
            TimerAction(
                period=3.0,
                actions=[left_effort_spawner, right_effort_spawner]
            ),
            delayed_nodes,
            # Gripper bridge (run keyboard controller in separate terminal)
            delayed_gripper_nodes,
        ]
    )

