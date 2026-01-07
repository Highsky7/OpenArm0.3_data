from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    port_arg = DeclareLaunchArgument('port', default_value='/dev/ttyACM2')
    baud_arg = DeclareLaunchArgument('baud', default_value='9600')

    cfg = os.path.join(get_package_share_directory('openarm_arduino_bridge'),'config','bridge.yaml')

    return LaunchDescription([
        port_arg, baud_arg,
        Node(
            package='openarm_arduino_bridge',
            executable='servo_bridge',
            name='openarm_arduino_bridge',
            output='screen',
            parameters=[cfg, {'port': LaunchConfiguration('port'),
                              'baud': LaunchConfiguration('baud')}]
        )
    ])
