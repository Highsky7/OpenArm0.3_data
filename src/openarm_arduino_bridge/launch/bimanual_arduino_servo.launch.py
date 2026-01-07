from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    port_arg = DeclareLaunchArgument('port', default_value='auto')
    baud_arg = DeclareLaunchArgument('baud', default_value='115200')

    return LaunchDescription([
        port_arg,
        baud_arg,
        Node(
            package='openarm_arduino_bridge',
            executable='bimanual_servo_bridge',
            name='openarm_bimanual_arduino_bridge',
            output='screen',
            parameters=[{
                'port': LaunchConfiguration('port'),
                'baud': LaunchConfiguration('baud'),
                # You can customize these parameters as needed
                # 'left_cmd_topic': '/left_gripper_cmd',
                # 'right_cmd_topic': '/right_gripper_cmd',
                # 'left_joint_name': 'left_gripper_joint',
                # 'right_joint_name': 'right_gripper_joint',
            }]
        )
    ])
