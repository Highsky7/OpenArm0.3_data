#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
LeRobot VLA Collection Launch File (Phase 2)

Part of 2-Phase VLA Data Collection Workflow.
This launch file replays trajectory from Phase 1 while recording camera observations.

=============================================================================
  Phase 2: Trajectory 재생 → VLA 데이터셋 생성 (카메라 포함)
=============================================================================

실행 방법:
  ros2 launch openarm_static_bimanual_bringup lerobot_vla_collection.launch.py \\
      trajectory_dataset:=~/lerobot_datasets/openarm_trajectory \\
      vla_dataset:=~/lerobot_datasets/openarm_vla

=============================================================================
  Launch 인자
=============================================================================
  trajectory_dataset   - Phase 1에서 생성한 trajectory 데이터셋 경로 (필수)
  vla_dataset          - 생성할 VLA 데이터셋 경로 (기본: trajectory + "_vla")
  episode_index        - 특정 에피소드만 처리 (-1 = 전체)
  playback_speed       - 재생 속도 (1.0 = 원본 속도)
  record_rate          - 녹화 프레임레이트 (Hz)

=============================================================================
  워크플로우
=============================================================================
  1. 카메라 초기화 대기 (약 10초)
  2. 로봇이 trajectory를 재생
  3. 동시에 카메라 observation 녹화
  4. 완료 후 VLA 데이터셋 자동 저장

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
            description='Path to static URDF file for Pinocchio'
        ),
        DeclareLaunchArgument(
            'active_arms',
            default_value='both',
            description="Which arms to control: 'left', 'right', or 'both'"
        ),
        # ===== Phase 2 Arguments =====
        DeclareLaunchArgument(
            'trajectory_dataset',
            default_value='',
            description='Path to Phase 1 trajectory dataset (required)'
        ),
        DeclareLaunchArgument(
            'vla_dataset',
            default_value='',
            description='Path for output VLA dataset (default: trajectory + "_vla")'
        ),
        DeclareLaunchArgument(
            'episode_index',
            default_value='-1',
            description='Specific episode to process (-1 = all episodes)'
        ),
        DeclareLaunchArgument(
            'playback_speed',
            default_value='1.0',
            description='Playback speed multiplier'
        ),
        DeclareLaunchArgument(
            'record_rate',
            default_value='30.0',
            description='Recording frame rate in Hz'
        ),
        DeclareLaunchArgument(
            'task_description',
            default_value='bimanual manipulation task',
            description='Language instruction for VLA training'
        ),
        DeclareLaunchArgument(
            'resume',
            default_value='true',
            description='Resume (append) to existing VLA dataset if true'
        ),
        DeclareLaunchArgument(
            'enable_initial_move',
            default_value='true',
            description='Enable moving to initial position before replay'
        ),
        DeclareLaunchArgument(
            'initial_move_duration',
            default_value='3.0',
            description='Duration to move to initial position (seconds)'
        ),
        DeclareLaunchArgument(
            'repeat_count',
            default_value='1',
            description='Number of times to repeat trajectory for creating multiple VLA episodes'
        ),
    ]
    
    use_mock_hardware = LaunchConfiguration('use_mock_hardware')
    can_device = LaunchConfiguration('can_device')
    urdf_path = LaunchConfiguration('urdf_path')
    active_arms = LaunchConfiguration('active_arms')
    trajectory_dataset = LaunchConfiguration('trajectory_dataset')
    vla_dataset = LaunchConfiguration('vla_dataset')
    episode_index = LaunchConfiguration('episode_index')
    playback_speed = LaunchConfiguration('playback_speed')
    record_rate = LaunchConfiguration('record_rate')
    task_description = LaunchConfiguration('task_description')
    resume = LaunchConfiguration('resume')
    enable_initial_move = LaunchConfiguration('enable_initial_move')
    initial_move_duration = LaunchConfiguration('initial_move_duration')
    repeat_count = LaunchConfiguration('repeat_count')
    
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
    
    # ===== Gravity Compensation Node (with replay mode) =====
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
            'enable_replay_mode': True,
            'external_cmd_timeout': 0.5,
            'enable_initial_move': enable_initial_move,
            'initial_move_duration': initial_move_duration,
            # Default initial positions: [pi/2, pi/6, -pi/2, pi/3, 0, pi/2, 0, 0]
            'initial_left_position': [1.5708, 0.5236, -1.5708, 1.0472, 0.0, 1.5708, 0.0, 0.0],
            # Default initial positions: [-pi/2, pi/6, pi/2, pi/3, 0, pi/2, 0, 0]
            'initial_right_position': [-1.5708, 0.5236, 1.5708, 1.0472, 0.0, 1.5708, 0.0, 0.0],
        }],
    )
    
    # ===== VLA Replay Recorder Node =====
    replay_recorder_node = Node(
        package='openarm_static_bimanual_bringup',
        executable='lerobot_vla_replay_recorder.py',
        name='lerobot_vla_replay_recorder',
        output='screen',
        parameters=[{
            'trajectory_dataset': trajectory_dataset,
            'vla_dataset': vla_dataset,
            'episode_index': episode_index,
            'playback_speed': playback_speed,
            'record_rate': record_rate,
            'task_description': task_description,
            'resume': resume,
            'repeat_count': repeat_count,
        }],
    )
    
    # ===== Info Messages =====
    info_message = LogInfo(
        msg="\n" + "="*70 + "\n" +
            "  🎬 Phase 2: VLA Data Collection 시작!\n" +
            "="*70 + "\n" +
            "  📌 Trajectory 재생 + 카메라 녹화 모드\n" +
            "  📌 카메라 초기화 대기 중... (약 10초)\n" +
            "  📌 준비 완료 후 자동으로 재생 + 녹화 시작\n" +
            "="*70
    )
    
    # ===== Delayed start =====
    delayed_effort_spawners = TimerAction(
        period=3.0,
        actions=[
            left_effort_spawner,
            right_effort_spawner,
        ]
    )
    
    delayed_gravity_comp = TimerAction(
        period=5.0,
        actions=[
            gravity_comp_node,
            info_message,
        ]
    )
    
    # Give cameras time to initialize before starting replay+record
    delayed_replay_recorder = TimerAction(
        period=10.0,
        actions=[
            replay_recorder_node,
        ]
    )
    
    return LaunchDescription(
        declared_arguments + [
            urdf_gen_process,
            base_launch,
            delayed_effort_spawners,
            delayed_gravity_comp,
            delayed_replay_recorder,
        ]
    )
