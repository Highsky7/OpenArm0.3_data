#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
LeRobot VLA Recording Launch File

Launches gravity compensation mode with LeRobot VLA data recorder.
Run keyboard_gripper_controller.py in a SEPARATE terminal for gripper control.

Usage:
    [Terminal 1] ros2 launch openarm_static_bimanual_bringup lerobot_vla_recording.launch.py
    [Terminal 2] ros2 run openarm_static_bimanual_bringup keyboard_gripper_controller.py

Recording Controls (Terminal 1):
    'r' - Start new episode
    's' - Stop and save episode
    'q' - Finalize dataset and quit
"""
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
    ExecuteProcess,
)
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
            'active_arms',
            default_value='both',
            description="Which arms to control: 'left', 'right', or 'both'"
        ),
        DeclareLaunchArgument(
            'record_rate',
            default_value='50.0',
            description='Recording rate in Hz'
        ),
        DeclareLaunchArgument(
            'dataset_name',
            default_value='openarm_bimanual',
            description='Name of the dataset to create'
        ),
        DeclareLaunchArgument(
            'save_dir',
            default_value='~/lerobot_datasets',
            description='Directory to save LeRobot datasets'
        ),
    ]
    
    use_mock_hardware = LaunchConfiguration('use_mock_hardware')
    can_device = LaunchConfiguration('can_device')
    urdf_path = LaunchConfiguration('urdf_path')
    active_arms = LaunchConfiguration('active_arms')
    record_rate = LaunchConfiguration('record_rate')
    dataset_name = LaunchConfiguration('dataset_name')
    save_dir = LaunchConfiguration('save_dir')
    
    pkg_share = FindPackageShare('openarm_static_bimanual_bringup')
    description_pkg_share = FindPackageShare('openarm_static_bimanual_description')
    
    # ===== Generate URDF for Pinocchio =====
    urdf_gen_process = ExecuteProcess(
        cmd=[
            'bash', '-c',
            [
                FindExecutable(name='xacro'), ' ',
                PathJoinSubstitution([description_pkg_share, 'urdf', 'openarm_static_bimanual.urdf.xacro']), ' ',
                'use_grippers:=true ',
                'use_mock_hardware:=', use_mock_hardware, ' ',
                'mount_half_x:=0.30 ',
                '-o ', urdf_path
            ]
        ],
        output='screen'
    )
    
    # ===== Include base bringup launch =====
    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_share, 'launch', 'sbopenarm.launch.py'])
        ),
        launch_arguments={
            'use_mock_hardware': use_mock_hardware,
            'can_device': can_device,
            'disable_torque': 'false',
            'active_mode': 'teleop',
            'rviz': 'true',
            'use_grippers': 'true',
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
            'safety_margin': 0.087,
            'limit_spring_k': 3.0,
            'active_arms': active_arms,
            'gravity_scale_joints': [0.5, 2.0, 1.1, 1.0, 1.5, 1.85, 1.65],
        }],
    )
    
    # ===== LeRobot VLA Recorder Node =====
    lerobot_recorder_node = Node(
        package='openarm_static_bimanual_bringup',
        executable='lerobot_vla_recorder.py',
        name='lerobot_vla_recorder',
        output='screen',
        prefix='xterm -e',  # Run in separate terminal for keyboard input
        parameters=[{
            'record_rate': record_rate,
            'dataset_name': dataset_name,
            'save_dir': save_dir,
            'robot_type': 'openarm_static_bimanual',
        }],
    )
    
    # ===== Delayed start for gravity comp and recorder =====
    delayed_nodes = TimerAction(
        period=5.0,
        actions=[
            gravity_comp_node,
            lerobot_recorder_node,
        ]
    )
    
    return LaunchDescription(
        declared_arguments + [
            urdf_gen_process,
            base_launch,
            TimerAction(
                period=3.0,
                actions=[
                    left_effort_spawner,
                    right_effort_spawner,
                ]
            ),
            delayed_nodes,
        ]
    )
