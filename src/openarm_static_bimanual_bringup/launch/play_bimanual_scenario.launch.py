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
        'send_delay', default_value='2.0',
        description='Seconds to wait before sending trajectory goal'
    )

    # 아두이노 브리지 관련
    servo_port = DeclareLaunchArgument('servo_port', default_value='auto')
    servo_baud = DeclareLaunchArgument('servo_baud', default_value='115200')

    auto_shutdown = DeclareLaunchArgument('auto_shutdown', default_value='true')
    shutdown_after = DeclareLaunchArgument('shutdown_after', default_value='50.0')

    # LaunchConfiguration 바인딩
    l_send_delay = LaunchConfiguration('send_delay')
    l_servo_port = LaunchConfiguration('servo_port')
    l_servo_baud = LaunchConfiguration('servo_baud')
    l_auto_shutdown = LaunchConfiguration('auto_shutdown')
    l_shutdown_after = LaunchConfiguration('shutdown_after')

    # ====== (1) 왼팔 트래젝토리 전송 ======
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
                            "positions": [1.3, -0.2, -1.571, 1.7, -0.2, 0.95, 0.2],
                            "time_from_start": {"sec": 6}
                        },
                        {
                            "positions": [1.3, -0.25, -1.571, 1.75, -0.2, 0.95, 0.2],
                            "time_from_start": {"sec": 8}
                        },
                        {
                            "positions": [1.3, -0.25, -1.571, 1.75, -0.2, 0.95, 0.2],
                            "time_from_start": {"sec": 10}
                        },
                        {
                            "positions": [0.0, 0.62, -1.571, 2.0, 0.0, -0.1, 0.0],
                            "time_from_start": {"sec": 16}
                        },
                        {
                            "positions": [0.0, 0.62, -1.571, 2.0, 0.0, -0.1, 0.0],
                            "time_from_start": {"sec": 32}
                        },
                        {
                            "positions": [1.3, -0.2, -1.571, 1.7, -0.2, 1.05, 0.2],
                            "time_from_start": {"sec": 38}
                        },
                        {
                            "positions": [1.3, -0.2, -1.571, 1.7, -0.2, 1.05, 0.2],
                            "time_from_start": {"sec": 40}
                        },
                        {
                            "positions": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                            "time_from_start": {"sec": 46}
                        }
                    ]
                }
            }'''
        ],
        output='screen'
    )
    delayed_send_left_traj = TimerAction(period=l_send_delay, actions=[send_left_traj])

    # ====== (2) 오른팔 트래젝토리 전송 ======
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
                            "positions": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                            "time_from_start": {"sec": 8}
                        },
                        {
                            "positions": [0.0, 0.95, 1.571, 2.2, 1.571, 0.2, 0.3],
                            "time_from_start": {"sec": 16}
                        },
                        {
                            "positions": [0.0, 0.53, 1.571, 1.9, 1.571, 0.2, 0.2],
                            "time_from_start": {"sec": 20}
                        },
                        {
                            "positions": [0.0, 0.53, 1.571, 1.9, 1.571, 0.2, 0.2],
                            "time_from_start": {"sec": 22}
                        },
                        {
                            "positions": [0.0, 0.53, 1.571, 1.92, 0.0, -0.2, 0.15],
                            "time_from_start": {"sec": 26}
                        },
                        {
                            "positions": [0.0, 0.53, 1.571, 1.92, 0.0, -0.22, 0.15],
                            "time_from_start": {"sec": 28}
                        },
                        {
                            "positions": [0.0, 0.95, 1.571, 2.2, 0.0, 0.25, 0.3],
                            "time_from_start": {"sec": 32}
                        },
                        {
                            "positions": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                            "time_from_start": {"sec": 44}
                        }
                    ]
                }
            }'''
        ],
        output='screen'
    )
    delayed_send_right_traj = TimerAction(period=l_send_delay, actions=[send_right_traj])

    # ====== (NEW) 그리퍼 명령 ======
    send_left_grip = TimerAction(
        period=10.5,
        actions=[ExecuteProcess(
            cmd=['ros2', 'topic', 'pub', '--once', '/left_gripper_cmd', 'std_msgs/msg/Float64', '{data: 1.0}'],
            output='screen'
        )]
    )

    send_left_release = TimerAction(
        period=41.0,
        actions=[ExecuteProcess(
            cmd=['ros2', 'topic', 'pub', '--once', '/left_gripper_cmd', 'std_msgs/msg/Float64', '{data: 0.0}'],
            output='screen'
        )]
    )

    send_right_grip = TimerAction(
        period=22.5,
        actions=[ExecuteProcess(
            cmd=['ros2', 'topic', 'pub', '--once', '/right_gripper_cmd', 'std_msgs/msg/Float64', '{data: 0.8}'],
            output='screen'
        )]
    )

    send_right_release = TimerAction(
        period=29.5,
        actions=[ExecuteProcess(
            cmd=['ros2', 'topic', 'pub', '--once', '/right_gripper_cmd', 'std_msgs/msg/Float64', '{data: 0.0'
            '}'],
            output='screen'
        )]
    )

    # ====== (NEW) 그리퍼 브릿지 노드 실행 ======
    gripper_bridge_node = Node(
        package='openarm_arduino_bridge',
        executable='bimanual_servo_bridge',
        output='screen',
        parameters=[{'port': l_servo_port, 'baud': l_servo_baud}]
    )

    # ====== (3) 자동 종료 ======
    final_shutdown = TimerAction(
        period=l_shutdown_after,
        actions=[EmitEvent(event=Shutdown())],
        condition=IfCondition(l_auto_shutdown)
    )

    return LaunchDescription([
        # 인자 선언
        send_delay,
        servo_port,
        servo_baud,
        auto_shutdown, 
        shutdown_after,

        # 그리퍼 브릿지 노드
        gripper_bridge_node,

        # 실행
        delayed_send_left_traj,
        delayed_send_right_traj,
        
        # 그리퍼 명령 추가
        send_left_grip,
        send_left_release,
        send_right_grip,
        send_right_release,

        final_shutdown
    ])
