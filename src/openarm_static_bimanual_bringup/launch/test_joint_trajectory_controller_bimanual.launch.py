# SPDX-License-Identifier: Apache-2.0

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_args = [
        DeclareLaunchArgument(
            "runtime_config_package",
            default_value="openarm_static_bimanual_bringup",
            description="Package that contains test goal yaml(s) under config/"
        ),
        DeclareLaunchArgument(
            "left_goals_yaml",
            default_value="test_jtc_left.yaml",
            description="YAML for left_joint_trajectory_controller publisher"
        ),
        DeclareLaunchArgument(
            "right_goals_yaml",
            default_value="test_jtc_right.yaml",
            description="YAML for right_joint_trajectory_controller publisher"
        ),
    ]

    runtime_pkg = LaunchConfiguration("runtime_config_package")
    left_yaml_name = LaunchConfiguration("left_goals_yaml")
    right_yaml_name = LaunchConfiguration("right_goals_yaml")

    left_goals_yaml = PathJoinSubstitution([FindPackageShare(runtime_pkg), "config", left_yaml_name])
    right_goals_yaml = PathJoinSubstitution([FindPackageShare(runtime_pkg), "config", right_yaml_name])

    left_pub = Node(
        package="ros2_controllers_test_nodes",
        executable="publisher_joint_trajectory_controller",
        name="publisher_left_joint_trajectory_controller",
        parameters=[left_goals_yaml],
        output={"stdout": "screen", "stderr": "screen"},
    )

    right_pub = Node(
        package="ros2_controllers_test_nodes",
        executable="publisher_joint_trajectory_controller",
        name="publisher_right_joint_trajectory_controller",
        parameters=[right_goals_yaml],
        output={"stdout": "screen", "stderr": "screen"},
    )

    return LaunchDescription(declared_args + [left_pub, right_pub])
