#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
LeRobot VLA Recording Launch File (Multi-Terminal Version)

This launch file starts the base robot infrastructure only.
Run gravity compensation, recorder, and gripper controller in SEPARATE terminals.

=============================================================================
  실행 방법 (3개의 터미널 필요)
=============================================================================

[Terminal 1] 기본 인프라 + 중력 보상 (이 launch 파일)
    ros2 launch openarm_static_bimanual_bringup lerobot_vla_recording.launch.py

[Terminal 2] 키보드 그리퍼 제어
    ros2 run openarm_static_bimanual_bringup keyboard_gripper_controller.py

[Terminal 3] 데이터 녹화
    ros2 run openarm_static_bimanual_bringup lerobot_vla_recorder.py

=============================================================================
  녹화 조작 (Terminal 3에서)
=============================================================================
    'r' - 새 에피소드 시작
    's' - 에피소드 저장 및 중지
    'q' - 데이터셋 저장 및 종료

=============================================================================
  Launch 인자
=============================================================================
    use_mock_hardware:=true/false  - 시뮬레이션 모드
    enable_cameras:=true/false     - 카메라 녹화 활성화
    can_device:=can0               - CAN 디바이스 이름
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
        DeclareLaunchArgument(
            'task_description',
            default_value='bimanual manipulation task',
            description='Language instruction for VLA training'
        ),
        DeclareLaunchArgument(
            'enable_cameras',
            default_value='true',
            description='Enable camera image recording'
        ),
    ]
    
    use_mock_hardware = LaunchConfiguration('use_mock_hardware')
    can_device = LaunchConfiguration('can_device')
    urdf_path = LaunchConfiguration('urdf_path')
    active_arms = LaunchConfiguration('active_arms')
    record_rate = LaunchConfiguration('record_rate')
    dataset_name = LaunchConfiguration('dataset_name')
    save_dir = LaunchConfiguration('save_dir')
    task_description = LaunchConfiguration('task_description')
    enable_cameras = LaunchConfiguration('enable_cameras')
    
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
    # 이 노드는 launch 파일에서 직접 실행됩니다 (Terminal 1)
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
            "  ✅ 기본 인프라 + 중력 보상 노드가 실행되었습니다!\n" +
            "="*70 + "\n" +
            "  다음 터미널에서 아래 명령어를 실행하세요:\n\n" +
            "  [Terminal 2] 키보드 그리퍼 제어:\n" +
            "    ros2 run openarm_static_bimanual_bringup keyboard_gripper_controller.py\n\n" +
            "  [Terminal 3] 데이터 녹화:\n" +
            "    ros2 run openarm_static_bimanual_bringup lerobot_vla_recorder.py\n" +
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
