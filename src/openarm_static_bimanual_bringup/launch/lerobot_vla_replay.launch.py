#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
LeRobot VLA Replay Launch File (with Gravity Compensation)

This launch file starts the robot infrastructure and replay node with gravity compensation.
Option A implementation: External position commands are sent to gravity_comp_node.

=============================================================================
  실행 방법
=============================================================================

[중력 보상 모드 - 권장]
  ros2 launch openarm_static_bimanual_bringup lerobot_vla_replay.launch.py \
      dataset_path:=/path/to/dataset.parquet \
      use_gravity_comp:=true

[직접 위치 제어 모드]
  ros2 launch openarm_static_bimanual_bringup lerobot_vla_replay.launch.py \
      dataset_path:=/path/to/dataset.parquet \
      use_gravity_comp:=false

=============================================================================
  Launch 인자
=============================================================================
  dataset_path         - 재생할 parquet 파일 경로 (필수)
  episode_index        - 특정 에피소드만 재생 (-1 = 전체)
  playback_speed       - 재생 속도 (1.0 = 원본 속도)
  loop                 - 반복 재생 여부 (true/false)
  use_action           - action 데이터 사용 여부 (false = observation.state 사용)
  use_gravity_comp     - 중력 보상 모드 사용 (true = 권장)
  use_mock_hardware    - 시뮬레이션 모드 (true/false)
"""
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
    ExecuteProcess,
    LogInfo,
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
            description='Path to static URDF file for Pinocchio'
        ),
        DeclareLaunchArgument(
            'active_arms',
            default_value='both',
            description="Which arms to control: 'left', 'right', or 'both'"
        ),
        # ===== Replay Arguments =====
        DeclareLaunchArgument(
            'dataset_path',
            default_value='',
            description='Path to the parquet dataset file (required)'
        ),
        DeclareLaunchArgument(
            'episode_index',
            default_value='-1',
            description='Specific episode to play (-1 = all episodes)'
        ),
        DeclareLaunchArgument(
            'playback_speed',
            default_value='1.0',
            description='Playback speed multiplier (1.0 = original speed)'
        ),
        DeclareLaunchArgument(
            'loop',
            default_value='false',
            description='Loop playback'
        ),
        DeclareLaunchArgument(
            'use_action',
            default_value='false',
            description='Use action data instead of observation.state'
        ),
        DeclareLaunchArgument(
            'use_gravity_comp',
            default_value='true',
            description='Use gravity compensation mode (recommended)'
        ),
    ]
    
    use_mock_hardware = LaunchConfiguration('use_mock_hardware')
    can_device = LaunchConfiguration('can_device')
    urdf_path = LaunchConfiguration('urdf_path')
    active_arms = LaunchConfiguration('active_arms')
    dataset_path = LaunchConfiguration('dataset_path')
    episode_index = LaunchConfiguration('episode_index')
    playback_speed = LaunchConfiguration('playback_speed')
    loop = LaunchConfiguration('loop')
    use_action = LaunchConfiguration('use_action')
    use_gravity_comp = LaunchConfiguration('use_gravity_comp')
    
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
    
    # ===== Effort controller spawners (for gravity compensation) =====
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
    
    # ===== Gravity Compensation Node (with replay mode enabled) =====
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
            # Option A: Enable external position command mode for replay
            'enable_replay_mode': True,
            'external_cmd_timeout': 0.5,
        }],
        condition=IfCondition(use_gravity_comp),
    )
    
    # ===== Replay Node =====
    replay_node = Node(
        package='openarm_static_bimanual_bringup',
        executable='lerobot_vla_replay.py',
        name='lerobot_vla_replay',
        output='screen',
        parameters=[{
            'dataset_path': dataset_path,
            'episode_index': episode_index,
            'playback_speed': playback_speed,
            'loop': loop,
            'use_action': use_action,
            'use_gravity_comp': use_gravity_comp,
        }],
    )
    
    # ===== Info Messages =====
    info_gravity_comp = LogInfo(
        msg="\n" + "="*70 + "\n" +
            "  🎮 중력 보상 Replay 모드로 시작합니다!\n" +
            "="*70 + "\n" +
            "  ✅ 중력 보상이 활성화되어 로봇이 자연스럽게 움직입니다.\n" +
            "  ✅ 외부 위치 명령 모드가 활성화되었습니다.\n" +
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
            info_gravity_comp,
        ]
    )
    
    delayed_replay = TimerAction(
        period=7.0,
        actions=[
            replay_node,
        ]
    )
    
    return LaunchDescription(
        declared_arguments + [
            urdf_gen_process,
            base_launch,
            delayed_effort_spawners,
            delayed_gravity_comp,
            delayed_replay,
        ]
    )
