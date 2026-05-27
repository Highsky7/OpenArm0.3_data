# SPDX-License-Identifier: Apache-2.0

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument("teleop_config", default_value="teleop_serial.yaml"),
        DeclareLaunchArgument("serial_port", default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("serial_baud", default_value="500000"),
    ]

    teleop_config = LaunchConfiguration("teleop_config")
    serial_port = LaunchConfiguration("serial_port")
    serial_baud = LaunchConfiguration("serial_baud")

    teleop_yaml = PathJoinSubstitution(
        [FindPackageShare("openarm_static_bimanual_bringup"), "config", teleop_config]
    )

    teleop_node = Node(
        package="openarm_static_bimanual_bringup",
        executable="teleop_serial_bridge.py",
        name="teleop_serial_bridge",
        output="screen",
        parameters=[
            teleop_yaml,
            {
                "serial_port": serial_port,
                "serial_baud": serial_baud,
            },
        ],
    )

    return LaunchDescription(declared_arguments + [teleop_node])
