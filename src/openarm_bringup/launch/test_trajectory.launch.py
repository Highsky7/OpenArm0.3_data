#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import TimerAction, ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node   # ← 추가: 브리지 노드 실행용
from launch.actions import EmitEvent
from launch.events import Shutdown
from launch.conditions import IfCondition

def generate_launch_description():
    # ====== 인자 선언 ======
    # 기존: 트래젝토리 보내기 지연
    send_delay = DeclareLaunchArgument(
        'send_delay', default_value='3.0',
        description='Seconds to wait before sending trajectory goal'
    )

    # 서보(아두이노) 관련
    servo_port = DeclareLaunchArgument('servo_port', default_value='/dev/ttyACM0')
    servo_baud = DeclareLaunchArgument('servo_baud', default_value='9600')

    # 서보 명령 타이밍/각도
    angle1_at = DeclareLaunchArgument('angle1_at', default_value='1.0')
    angle1    = DeclareLaunchArgument('angle1',    default_value='-50')
    angle2_at = DeclareLaunchArgument('angle2_at', default_value='11.0')
    angle2    = DeclareLaunchArgument('angle2',    default_value='0')
    angle3_at = DeclareLaunchArgument('angle3_at', default_value='27.0')
    angle3    = DeclareLaunchArgument('angle3',    default_value='-50')
    auto_shutdown = DeclareLaunchArgument('auto_shutdown', default_value='true')
    shutdown_after = DeclareLaunchArgument('shutdown_after', default_value='45.0')
   

    # LaunchConfiguration 바인딩
    l_send_delay = LaunchConfiguration('send_delay')
    l_servo_port = LaunchConfiguration('servo_port')
    l_servo_baud = LaunchConfiguration('servo_baud')
    l_angle1_at, l_angle1 = LaunchConfiguration('angle1_at'), LaunchConfiguration('angle1')
    l_angle2_at, l_angle2 = LaunchConfiguration('angle2_at'), LaunchConfiguration('angle2')
    l_angle3_at, l_angle3 = LaunchConfiguration('angle3_at'), LaunchConfiguration('angle3')
    l_auto_shutdown = LaunchConfiguration('auto_shutdown')
    l_shutdown_after = LaunchConfiguration('shutdown_after')

    # ====== (1) 아두이노 브리지 노드 실행 ======
    servo_bridge = Node(
        package='openarm_arduino_bridge',
        executable='servo_bridge',
        name='openarm_arduino_bridge',
        output='screen',
        parameters=[{'port': l_servo_port, 'baud': l_servo_baud}]
    )

    # ====== (2) 기존 트래젝토리 전송(딜레이 포함) ======
    send_traj = ExecuteProcess(
        cmd=[
            'ros2','action','send_goal',
            '/joint_trajectory_controller/follow_joint_trajectory',
            'control_msgs/action/FollowJointTrajectory',
            '{trajectory: {joint_names: ["rev1","rev2","rev3","rev4","rev5","rev6","rev7"], points: ['
            '{positions: [0, 0, 0, 0, 0, 0, 0], velocities: [0,0,0,0,0,0,0], time_from_start: {sec: 1}}, '
            '{positions: [0, 0, 1.5708, 1.2, 0, 1.34, 0.1], velocities: [0,0,0,0,0,0,0], time_from_start: {sec: 4}}, '
            '{positions: [0, 0.5, 1.5708, 1.2, 0, 1.2, 0.1], velocities: [0,0,0,0,0,0,0], time_from_start: {sec: 6}}, '
            '{positions: [0, 0.5, 1.5708, 1.2, 0, 1.2, 0.1], velocities: [0,0,0,0,0,0,0], time_from_start: {sec: 11}}, '
            '{positions: [0, -0.5, 1.5708, 1.2, 0, 1.34, 0.1], velocities: [0,0,0,0,0,0,0], time_from_start: {sec: 15}},'
            '{positions: [-1.5708, -0.5, 1.5708, 1.2, 0, 1.34, 0.1], velocities: [0,0,0,0,0,0,0], time_from_start: {sec: 19}},'
            '{positions: [-1.5708, 0.5, 1.5708, 1.2, 0, 1.34, 0.1], velocities: [0,0,0,0,0,0,0], time_from_start: {sec: 22}},'
            '{positions: [-1.5708, 0.5, 1.5708, 1.2, 0, 1.34, 0.1], velocities: [0,0,0,0,0,0,0], time_from_start: {sec: 30}},'
            '{positions: [-1.5708, -0.5, 1.5708, 1.2, 0, 1.34, 0.1], velocities: [0,0,0,0,0,0,0], time_from_start: {sec: 32}},'
            '{positions: [0, 0, 0, 0, 0, 0, 0], velocities: [0,0,0,0,0,0,0], time_from_start: {sec: 35}}'
            ']}}'
        ],
        output='screen'
    )
    delayed_send_traj = TimerAction(period=l_send_delay, actions=[send_traj])

    # ====== (3) 서보 쪽 단발 명령들(원하는 시점에 1번만) ======
    send_angle1 = TimerAction(
        period=l_angle1_at,
        actions=[ExecuteProcess(
            cmd=['ros2','topic','pub','--once','/servo_angle','std_msgs/Int32',
                 ['{data: ', l_angle1, '}']],
            output='screen'
        )]
    )
    send_angle2 = TimerAction(
        period=l_angle2_at,
        actions=[ExecuteProcess(
            
            cmd=['ros2','topic','pub','--once','/servo_angle','std_msgs/Int32',
                 ['{data: ', l_angle2, '}']],
            output='screen'
        )]
    )
    send_angle3 = TimerAction(
        period=l_angle3_at,
        actions=[ExecuteProcess(
            cmd=['ros2','topic','pub','--once','/servo_angle','std_msgs/Int32',
                 ['{data: ', l_angle3, '}']],
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
        angle1_at, angle1, angle2_at, angle2, angle3_at, angle3, 
        auto_shutdown, shutdown_after,

        # 실행
        servo_bridge,
        delayed_send_traj,
        send_angle1, send_angle2, send_angle3, final_shutdown  
    ])
