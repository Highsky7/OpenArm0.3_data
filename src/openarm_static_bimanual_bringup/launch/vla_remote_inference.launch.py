#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
VLA Remote Inference Launch File

SSH 터널을 통해 원격 VLA 서버와 통신하는 추론 노드를 실행합니다.

주의사항:
- initial_move 로직이 포함되어 있지 않습니다
- 이 launch 파일 실행 전에 lerobot_trajectory_recording.launch.py로 로봇 초기화 필요
- enable_control:=false (기본값)로 안전한 dry-run 테스트 가능

Author: Antigravity Assistant
Date: 2026-02-09
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Launch description 생성"""
    
    # Launch arguments
    server_port_arg = DeclareLaunchArgument(
        'server_port',
        default_value='5555',
        description='ZeroMQ 서버 포트 (SSH 터널 로컬 포트)'
    )
    
    inference_rate_arg = DeclareLaunchArgument(
        'inference_rate',
        default_value='10.0',
        description='추론 요청 주기 (Hz)'
    )
    
    enable_control_arg = DeclareLaunchArgument(
        'enable_control',
        default_value='false',
        description='로봇 제어 활성화 (false=dry-run, true=실제 제어)'
    )
    
    task_description_arg = DeclareLaunchArgument(
        'task_description',
        default_value='manipulation task',
        description='VLA 모델에 전달할 태스크 설명'
    )
    
    debug_arg = DeclareLaunchArgument(
        'debug',
        default_value='true',
        description='디버그 모드 (상세 로그 출력)'
    )
    
    image_size_arg = DeclareLaunchArgument(
        'image_size',
        default_value='256',
        description='이미지 리사이즈 크기'
    )
    
    timeout_ms_arg = DeclareLaunchArgument(
        'timeout_ms',
        default_value='5000',
        description='서버 응답 타임아웃 (ms)'
    )
    
    # VLA Remote Client Node
    vla_client_node = Node(
        package='openarm_static_bimanual_bringup',
        executable='vla_remote_client_node.py',
        name='vla_remote_client',
        output='screen',
        parameters=[{
            'server_port': LaunchConfiguration('server_port'),
            'inference_rate': LaunchConfiguration('inference_rate'),
            'enable_control': LaunchConfiguration('enable_control'),
            'task_description': LaunchConfiguration('task_description'),
            'debug': LaunchConfiguration('debug'),
            'image_size': LaunchConfiguration('image_size'),
            'timeout_ms': LaunchConfiguration('timeout_ms'),
        }],
    )
    
    return LaunchDescription([
        # Arguments
        server_port_arg,
        inference_rate_arg,
        enable_control_arg,
        task_description_arg,
        debug_arg,
        image_size_arg,
        timeout_ms_arg,
        # Nodes
        vla_client_node,
    ])
