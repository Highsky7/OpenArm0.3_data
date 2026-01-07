# SPDX-License-Identifier: Apache-2.0

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # === Args ===
    declared_args = [
        # bringup 패키지/파일 위치 (필요시 변경)
        DeclareLaunchArgument(
            "runtime_config_package",
            default_value="openarm_static_bimanual_bringup",
            description="Package that contains test goal yaml(s) under config/"
        ),
        # 테스트 목표값 파일 (좌/우 각각)
        DeclareLaunchArgument(
            "left_goals_yaml",
            default_value="test_goal_publishers_left.yaml",
            description="YAML for left_forward_position_controller publisher"
        ),
        DeclareLaunchArgument(
            "right_goals_yaml",
            default_value="test_goal_publishers_right.yaml",
            description="YAML for right_forward_position_controller publisher"
        ),
        # 컨트롤러 매니저 네임스페이스
        DeclareLaunchArgument(
            "controller_manager_ns",
            default_value="/controller_manager",
            description="Controller manager node name or namespace"
        ),
    ]

    runtime_pkg = LaunchConfiguration("runtime_config_package")
    left_yaml_name = LaunchConfiguration("left_goals_yaml")
    right_yaml_name = LaunchConfiguration("right_goals_yaml")
    cm_ns = LaunchConfiguration("controller_manager_ns")

    # === Files ===
    left_goals_yaml = PathJoinSubstitution([FindPackageShare(runtime_pkg), "config", left_yaml_name])
    right_goals_yaml = PathJoinSubstitution([FindPackageShare(runtime_pkg), "config", right_yaml_name])

    # === Spawners ===
    # (주의) 동일 조인트를 쓰는 컨트롤러는 동시에 활성화될 수 없음.
    # bringup에서 JTC가 이미 active라면 먼저 unspawner로 내려주거나, inactive 상태에서 시작해줘.
    sp_left_fpc = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["left_forward_position_controller", "-c", cm_ns],
        output="screen",
    )
    sp_right_fpc = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["right_forward_position_controller", "-c", cm_ns],
        output="screen",
    )

    # === Publishers (ros2_controllers_test_nodes) ===
    pub_left = Node(
        package="ros2_controllers_test_nodes",
        executable="publisher_forward_position_controller",
        name="publisher_left_forward_position_controller",
        parameters=[left_goals_yaml],
        output={"stdout": "screen", "stderr": "screen"},
    )
    pub_right = Node(
        package="ros2_controllers_test_nodes",
        executable="publisher_forward_position_controller",
        name="publisher_right_forward_position_controller",
        parameters=[right_goals_yaml],
        output={"stdout": "screen", "stderr": "screen"},
    )

    # === Sequence ===
    # 1) left/right FPC 스폰을 약간 지연
    delay_spawn = TimerAction(period=0.5, actions=[sp_left_fpc, sp_right_fpc])
    # 2) 오른쪽 FPC 스폰이 끝난 뒤 퍼블리셔 둘을 시작
    start_publishers_after_right = RegisterEventHandler(
        OnProcessExit(target_action=sp_right_fpc, on_exit=[pub_left, pub_right])
    )

    return LaunchDescription(declared_args + [delay_spawn, start_publishers_after_right])
