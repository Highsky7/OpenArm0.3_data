#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import TimerAction, ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import EmitEvent
from launch.events import Shutdown
from launch.conditions import IfCondition

def generate_launch_description():
    # ====== 인자 선언 ======
    send_delay = DeclareLaunchArgument(
        'send_delay', default_value='10.0',
        description='Seconds to wait before sending trajectory goal'
    )

    # 아두이노 브리지 관련
    servo_port = DeclareLaunchArgument('servo_port', default_value='auto')
    servo_baud = DeclareLaunchArgument('servo_baud', default_value='115200')

    # 그리퍼 제어 타이밍/값
    left_grip_at = DeclareLaunchArgument('left_grip_at', default_value='5.0')
    left_grip_val = DeclareLaunchArgument('left_grip_val', default_value='1.0') # 닫기

    right_grip_at = DeclareLaunchArgument('right_grip_at', default_value='10.0')
    right_grip_val = DeclareLaunchArgument('right_grip_val', default_value='0.5') # 반만 열기

    left_release_at = DeclareLaunchArgument('left_release_at', default_value='15.0')
    left_release_val = DeclareLaunchArgument('left_release_val', default_value='0.0') # 열기

    auto_shutdown = DeclareLaunchArgument('auto_shutdown', default_value='true')
    shutdown_after = DeclareLaunchArgument('shutdown_after', default_value='45.0')

    # LaunchConfiguration 바인딩
    l_send_delay = LaunchConfiguration('send_delay')
    l_servo_port = LaunchConfiguration('servo_port')
    l_servo_baud = LaunchConfiguration('servo_baud')
    l_left_grip_at, l_left_grip_val = LaunchConfiguration('left_grip_at'), LaunchConfiguration('left_grip_val')
    l_right_grip_at, l_right_grip_val = LaunchConfiguration('right_grip_at'), LaunchConfiguration('right_grip_val')
    l_left_release_at, l_left_release_val = LaunchConfiguration('left_release_at'), LaunchConfiguration('left_release_val')
    l_auto_shutdown = LaunchConfiguration('auto_shutdown')
    l_shutdown_after = LaunchConfiguration('shutdown_after')

    # ====== (1) 양팔 아두이노 브리지 노드 실행 ======
    bimanual_servo_bridge = Node(
        package='openarm_arduino_bridge',
        executable='bimanual_servo_bridge',
        name='openarm_bimanual_arduino_bridge',
        output='screen',
        parameters=[{'port': l_servo_port, 'baud': l_servo_baud}]
    )

    # ====== (2) 왼팔 트래젝토리 전송 ======
    send_left_traj = ExecuteProcess(
        cmd=[
            'ros2', 'action', 'send_goal',
            '/left_joint_trajectory_controller/follow_joint_trajectory',
            'control_msgs/action/FollowJointTrajectory',
            '''{
                "trajectory": {
                    "joint_names": [
                        "left_rev1", "left_rev2", "left_rev3", "left_rev4", "left_rev5", "left_rev6", "left_rev7"
                    ],
                    "points": [
                        {
                            "positions": [0,0,0,0,0,0,0],
                            "time_from_start": {"sec": 1}
                        },
                        {
                            "positions": [0,0,1.57,1.2,0,1.34,0],
                            "time_from_start": {"sec": 4}
                        },
                        {
                            "positions": [0,0.5,1.57,1.2,0,1.2,0],
                            "time_from_start": {"sec": 8}
                        },
                        {
                            "positions": [0,-0.5,1.57,1.2,0,1.34,0],
                            "time_from_start": {"sec": 12}
                        },
                        {
                            "positions": [0,0,0,0,0,0,0],
                            "time_from_start": {"sec": 16}
                        }
                    ]
                }
            }'''
        ],
        output='screen'
    )
    delayed_send_left_traj = TimerAction(period=l_send_delay, actions=[send_left_traj])

    # ====== (3) 오른팔 트래젝토리 전송 ======
    send_right_traj = ExecuteProcess(
        cmd=[
            'ros2', 'action', 'send_goal',
            '/right_joint_trajectory_controller/follow_joint_trajectory',
            'control_msgs/action/FollowJointTrajectory',
            '''{
                "trajectory": {
                    "joint_names": [
                        "right_rev1", "right_rev2", "right_rev3", "right_rev4", "right_rev5", "right_rev6", "right_rev7"
                    ],
                    "points": [
                        {
                            "positions": [0,0,0,0,0,0,0],
                            "time_from_start": {"sec": 1}
                        },
                        {
                            "positions": [0,0.5,0,0,0,0,0],
                            "time_from_start": {"sec": 4}
                        },
                        {
                            "positions": [0,0.5,0.5,0,0,0,0],
                            "time_from_start": {"sec": 8}
                        },
                        {
                            "positions": [0,0.5,-0.5,0,0,0,0],
                            "time_from_start": {"sec": 12}
                        },
                        {
                            "positions": [0,0,0,0,0,0,0],
                            "time_from_start": {"sec": 16}
                        }
                    ]
                }
            }'''
        ],
        output='screen'
    )
    delayed_send_right_traj = TimerAction(period=l_send_delay, actions=[send_right_traj])

    # ====== (4) 양팔 그리퍼 명령들 ======
    send_left_grip = TimerAction(
        period=l_left_grip_at,
        actions=[ExecuteProcess(
            cmd=['ros2', 'topic', 'pub', '--once', '/left_gripper_cmd', 'std_msgs/msg/Float64',
                 ['{data: ', l_left_grip_val, '}']],
            output='screen'
        )]
    )

    send_right_grip = TimerAction(
        period=l_right_grip_at,
        actions=[ExecuteProcess(
            cmd=['ros2', 'topic', 'pub', '--once', '/right_gripper_cmd', 'std_msgs/msg/Float64',
                 ['{data: ', l_right_grip_val, '}']],
            output='screen'
        )]
    )

    send_left_release = TimerAction(
        period=l_left_release_at,
        actions=[ExecuteProcess(
            cmd=['ros2', 'topic', 'pub', '--once', '/left_gripper_cmd', 'std_msgs/msg/Float64',
                 ['{data: ', l_left_release_val, '}']],
            output='screen'
        )]
    )

    final_shutdown = TimerAction(
        period=l_shutdown_after,
        actions=[EmitEvent(event=Shutdown())],
        condition=IfCondition(l_auto_shutdown)
    )

    return LaunchDescription([
        # 인자 선언
        send_delay, servo_port, servo_baud,
        left_grip_at, left_grip_val, right_grip_at, right_grip_val, left_release_at, left_release_val,
        auto_shutdown, shutdown_after,

        # 실행
        bimanual_servo_bridge,
        delayed_send_left_traj,
        delayed_send_right_traj,
        send_left_grip, send_right_grip, send_left_release,
        final_shutdown
    ])
