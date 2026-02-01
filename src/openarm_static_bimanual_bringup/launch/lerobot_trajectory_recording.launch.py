#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
LeRobot Trajectory Recording Launch File (Phase 1)

Part of 2-Phase VLA Data Collection Workflow.
This launch file sets up the robot for trajectory-only recording (NO cameras).

=============================================================================
  Phase 1: 수동 티칭 → Trajectory 녹화 (카메라 없음)
=============================================================================

[Terminal 1] 이 launch 파일 실행:
    ros2 launch openarm_static_bimanual_bringup lerobot_trajectory_recording.launch.py

[Terminal 2] 키보드 그리퍼 제어:
    ros2 run openarm_static_bimanual_bringup keyboard_gripper_controller.py

[Terminal 3] Trajectory 녹화:
    ros2 run openarm_static_bimanual_bringup lerobot_trajectory_recorder.py

=============================================================================
  녹화 조작 (Terminal 3에서)
=============================================================================
    'r' - 새 에피소드 시작
    's' - 에피소드 저장 및 중지
    'q' - 데이터셋 저장 및 종료

=============================================================================
  Phase 2로 이동
=============================================================================
    녹화 완료 후:
    ros2 launch openarm_static_bimanual_bringup lerobot_vla_collection.launch.py \\
      trajectory_dataset:=~/lerobot_datasets/openarm_trajectory

"""
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
    ExecuteProcess,
    LogInfo,
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
            default_value='30.0',
            description='Recording rate in Hz (higher without cameras)'
        ),
        DeclareLaunchArgument(
            'dataset_name',
            default_value='openarm_trajectory',
            description='Name of the trajectory dataset to create'
        ),
        DeclareLaunchArgument(
            'save_dir',
            default_value='~/lerobot_datasets',
            description='Directory to save LeRobot datasets'
        ),
        DeclareLaunchArgument(
            'task_description',
            default_value='bimanual manipulation task',
            description='Language instruction for VLA training'
        ),
    ]
    
    use_mock_hardware = LaunchConfiguration('use_mock_hardware')
    can_device = LaunchConfiguration('can_device')
    urdf_path = LaunchConfiguration('urdf_path')
    active_arms = LaunchConfiguration('active_arms')
    
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
            'gravity_scale_joints': [0.0, 2.5, 1.7, 1.7, 2.0, 2.0, 2.0],
        }],
    )
    
    # ===== 안내 메시지 =====
    info_message = LogInfo(
        msg="\n" + "="*70 + "\n" +
            "  ✅ Phase 1: Trajectory Recording 환경 준비 완료!\n" +
            "="*70 + "\n" +
            "  📌 카메라 녹화: 비활성화 (경량 trajectory 모드)\n\n" +
            "  다음 터미널에서 아래 명령어를 실행하세요:\n\n" +
            "  [Terminal 2] 키보드 그리퍼 제어:\n" +
            "    ros2 run openarm_static_bimanual_bringup keyboard_gripper_controller.py\n\n" +
            "  [Terminal 3] Trajectory 녹화:\n" +
            "    ros2 run openarm_static_bimanual_bringup lerobot_trajectory_recorder.py\n\n" +
            "  녹화 완료 후 Phase 2로 이동:\n" +
            "    ros2 launch openarm_static_bimanual_bringup lerobot_vla_collection.launch.py\n" +
            "="*70
    )
    
    # ===== Delayed start for gravity comp =====
    delayed_nodes = TimerAction(
        period=5.0,
        actions=[
            gravity_comp_node,
            info_message,
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
