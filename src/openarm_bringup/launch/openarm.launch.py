# Copyright 2025 Reazon Holdings, Inc.
# Copyright 2024 Stogl Robotics Consulting UG (haftungsbeschränkt)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

#
# Source of this file are templates in
# [RosTeamWorkspace](https://github.com/StoglRobotics/ros_team_workspace) repository.
#
# Author: Dr. Denis
#

from launch_ros.parameter_descriptions import ParameterValue

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "runtime_config_package",
            default_value="openarm_bringup",
            description='Package with the controller\'s configuration in "config" folder. \
        Usually the argument is not set, it enables use of a custom setup.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controllers_file",
            default_value="openarm_controllers.yaml",
            description="YAML file with the controllers configuration.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="openarm_description",
            description="Description package with robot URDF/xacro files. Usually the argument \
        is not set, it enables use of a custom description.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="openarm.urdf.xacro",
            description="URDF/XACRO description file with the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value="",
            description="Prefix of the joint names, useful for \
        multi-robot setup. If changed than also joint names in the controllers' configuration \
        have to be updated.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="false",
            description="Start robot with mock hardware mirroring command to its states.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "mock_sensor_commands",
            default_value="false",
            description="Enable mock command interfaces for sensors used for simple simulations. \
            Used only if 'use_mock_hardware' parameter is true.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_controller",
            default_value="forward_position_controller",
            choices=["forward_position_controller",
                     "joint_trajectory_controller",
                     "teleop_stream_controller"
                    ],
            description="Robot controller to start.",
        )
    )
    declared_arguments.append(
    	DeclareLaunchArgument("can_device", default_value="can0",
      		description="Linux CAN netdev name (e.g., can0, slcan0)"
        )
    )
    declared_arguments.append(
    	DeclareLaunchArgument("can_fd", default_value="false",
      		description="Use CAN-FD (false for CANable 2.0)"
        )
    )
    declared_arguments.append(
    	DeclareLaunchArgument("disable_torque", default_value="true",
      		description="Start with torque disabled for safety"
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument("enable_gripper_bridge", default_value="false")
    )
    declared_arguments.append(
        DeclareLaunchArgument("gripper_port", default_value="/dev/ttyACM1")
    )
    declared_arguments.append(
        DeclareLaunchArgument("gripper_baud", default_value="9600")
    )

    # Initialize Arguments
    runtime_config_package = LaunchConfiguration("runtime_config_package")
    controllers_file = LaunchConfiguration("controllers_file")
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    prefix = LaunchConfiguration("prefix")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    mock_sensor_commands = LaunchConfiguration("mock_sensor_commands")
    robot_controller = LaunchConfiguration("robot_controller")
    can_device = LaunchConfiguration("can_device")
    can_fd = LaunchConfiguration("can_fd")
    disable_torque = LaunchConfiguration("disable_torque")
    enable_gripper_bridge = LaunchConfiguration("enable_gripper_bridge")
    gripper_port = LaunchConfiguration("gripper_port")
    gripper_baud = LaunchConfiguration("gripper_baud")


    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(description_package),
                 "urdf", description_file]
            ),
            " ",
            "prefix:=",
            prefix,
            " ",
            "use_mock_hardware:=",
            use_mock_hardware,
            " ",
            "mock_sensor_commands:=",
            mock_sensor_commands,
            " ",
            "can_device:=", 
            can_device, 
            " ",
            "can_fd:=", 
            can_fd, 
            " ",
            "disable_torque:=", 
            disable_torque, 
            " ",    
        ]
    )

    robot_description = {
    "robot_description": ParameterValue(robot_description_content, value_type=str)
}

    robot_controllers = PathJoinSubstitution(
        [FindPackageShare(runtime_config_package), "config", controllers_file]
    )
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(description_package), "rviz",
         "robot_description.rviz"]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="both",
        parameters=[robot_controllers],  # ← robot_description 제거
        remappings=[('~/robot_description', '/robot_description')],  # ← 추가
    )


    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    # robot_state_pub_node = Node(
    # package="robot_state_publisher",
    # executable="robot_state_publisher",
    # output="both",
    # parameters=[robot_description],
    # remappings=[("/joint_states", "/joint_states/zeroed")],
    # )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster",
                   "--controller-manager", "/controller_manager"],
    )

    teleop_follower_node = Node(
        package="openarm_bringup",
        executable="teleop_follower.py",
        name="teleop_follower",
        output="screen",
        parameters=[{
            # 필요시 매핑/리밋 튜닝
            "idx_map": [0,1,2,3,4,5,6],
            "rate_hz": 200.0,
            "alpha": 0.15,
            "vel_limit": 2.0,
            # "min_limits": [-3.14,-1.57,-2.5,-2.5,-2.8,-2.8,-2.8],
            # "max_limits": [ 3.14, 1.57, 2.5, 2.5, 2.8, 2.8, 2.8],
        }],
    )

    robot_controller_names = [robot_controller]
    robot_controller_spawners = []
    for controller in robot_controller_names:
        robot_controller_spawners += [
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[controller, "-c", "/controller_manager"],
            )
        ]

    gripper_serial_bridge = Node(
        package="openarm_arduino_bridge",
        executable="servo_bridge",
        name="gripper_serial_bridge",
        output="screen",
        condition=IfCondition(enable_gripper_bridge),
        parameters=[{
            "port": gripper_port,
            "baud": gripper_baud,
            "joint_name": "left_pris1",
            "js_topic": "/joint_states",
            "write_min_period_ms": 200,
            "inject_when_absent": True,
            "absent_timeout_ms": 300,
            "default_when_off": 0.0,
        }],
    )


    # inactive_robot_controller_names = [
    #     "joint_trajectory_controller",
    #     "forward_position_controller"
    #     ]
    
    inactive_robot_controller_spawners = [
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_trajectory_controller", "-c", "/controller_manager", "--inactive"],
            condition=IfCondition(PythonExpression(['"', robot_controller, '" != "joint_trajectory_controller"'])),
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["forward_position_controller", "-c", "/controller_manager", "--inactive"],
            condition=IfCondition(PythonExpression(['"', robot_controller, '" != "forward_position_controller"'])),
        ),
        # (선택) teleop_stream_controller도 쓰면 여기도 추가
        # Node(... teleop_stream_controller ... condition=IfCondition(PythonExpression(['"', robot_controller, '" != "teleop_stream_controller"']))),
        ]

    # Delay loading and activation of `joint_state_broadcaster` after start of ros2_control_node
    delay_joint_state_broadcaster_spawner_after_ros2_control_node = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=control_node,
            on_start=[
                TimerAction(
                    period=5.0,
                    actions=[joint_state_broadcaster_spawner],
                ),
            ],
        )
    )

    # Delay loading and activation of robot_controller_names after `joint_state_broadcaster`
    delay_robot_controller_spawners_after_joint_state_broadcaster_spawner = []
    for i, controller in enumerate(robot_controller_spawners):
        delay_robot_controller_spawners_after_joint_state_broadcaster_spawner += [
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=(
                        robot_controller_spawners[i - 1]
                        if i > 0
                        else joint_state_broadcaster_spawner
                    ),
                    on_exit=[controller],
                )
            )
        ]

    # Delay start of inactive_robot_controller_names after other controllers
    delay_inactive_robot_controller_spawners_after_joint_state_broadcaster_spawner = []
    for i, controller in enumerate(inactive_robot_controller_spawners):
        delay_inactive_robot_controller_spawners_after_joint_state_broadcaster_spawner += [
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=(
                        inactive_robot_controller_spawners[i - 1]
                        if i > 0
                        else robot_controller_spawners[-1]
                    ),
                    on_exit=[controller],
                )
            )
        ]


    if robot_controller_spawners:
        last_spawner = robot_controller_spawners[-1]
    else:
        # 혹시 비어있다면 JSB 이후에라도 시작
        last_spawner = joint_state_broadcaster_spawner

    delay_teleop_follower_after_controllers = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=last_spawner,
            on_exit=[teleop_follower_node],
        )
    )

    return LaunchDescription(
        declared_arguments
        + [
            control_node,
            robot_state_pub_node,
            rviz_node,
            delay_joint_state_broadcaster_spawner_after_ros2_control_node,
            delay_teleop_follower_after_controllers, gripper_serial_bridge,
        ]
        + delay_robot_controller_spawners_after_joint_state_broadcaster_spawner
        + delay_inactive_robot_controller_spawners_after_joint_state_broadcaster_spawner
    )
