# SPDX-License-Identifier: Apache-2.0

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # ===== Args =====
    declared_arguments = [
        DeclareLaunchArgument("runtime_config_package", default_value="openarm_static_bimanual_bringup"),
        DeclareLaunchArgument("controllers_file",       default_value="openarm_static_bimanual_controllers.yaml"),
        DeclareLaunchArgument("description_package",    default_value="openarm_static_bimanual_description"),
        DeclareLaunchArgument("description_file",       default_value="openarm_static_bimanual.urdf.xacro"),

        DeclareLaunchArgument("zero_pos",        default_value="up"),
        DeclareLaunchArgument("left_zero_pos",   default_value=""),
        DeclareLaunchArgument("right_zero_pos",  default_value=""),
        DeclareLaunchArgument("mount_half_x",    default_value="0.30"),

        DeclareLaunchArgument("use_ros2_control",default_value="true"),
        DeclareLaunchArgument("can_device",      default_value="can0"),
        DeclareLaunchArgument("left_can_device", default_value=""),
        DeclareLaunchArgument("right_can_device",default_value=""),
        DeclareLaunchArgument("can_fd",          default_value="false"),
        DeclareLaunchArgument("disable_torque",  default_value="true"),
        DeclareLaunchArgument("use_mock_hardware", default_value="true"),

        DeclareLaunchArgument("active_mode", default_value="jtc"),  # jtc | fpc | teleop

        # 그리퍼(기본 비활성)
        DeclareLaunchArgument("enable_gripper_left",  default_value="false"),
        DeclareLaunchArgument("enable_gripper_right", default_value="false"),
        DeclareLaunchArgument("gripper_left_port",  default_value="/dev/ttyACM0"),
        DeclareLaunchArgument("gripper_right_port", default_value="/dev/ttyACM1"),
        DeclareLaunchArgument("gripper_baud", default_value="9600"),
        DeclareLaunchArgument("use_grippers", default_value="false"),  # URDF gripper joints

        # 시각화 관련: RViz 기본 ON, JSP GUI 기본 OFF
        DeclareLaunchArgument("gui",  default_value="false"),   # Joint State Publisher GUI
        DeclareLaunchArgument("rviz", default_value="true"),    # RViz
    ]

    # ===== LCs =====
    runtime_config_package = LaunchConfiguration("runtime_config_package")
    controllers_file = LaunchConfiguration("controllers_file")
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")

    zero_pos = LaunchConfiguration("zero_pos")
    left_zero_pos = LaunchConfiguration("left_zero_pos")
    right_zero_pos = LaunchConfiguration("right_zero_pos")
    mount_half_x = LaunchConfiguration("mount_half_x")

    use_ros2_control = LaunchConfiguration("use_ros2_control")
    can_device = LaunchConfiguration("can_device")
    left_can_device = LaunchConfiguration("left_can_device")
    right_can_device = LaunchConfiguration("right_can_device")
    can_fd = LaunchConfiguration("can_fd")
    disable_torque = LaunchConfiguration("disable_torque")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")

    active_mode = LaunchConfiguration("active_mode")
    gui  = LaunchConfiguration("gui")
    rviz = LaunchConfiguration("rviz")

    enable_gripper_left  = LaunchConfiguration("enable_gripper_left")
    enable_gripper_right = LaunchConfiguration("enable_gripper_right")
    gripper_left_port  = LaunchConfiguration("gripper_left_port")
    gripper_right_port = LaunchConfiguration("gripper_right_port")
    gripper_baud = LaunchConfiguration("gripper_baud")
    use_grippers = LaunchConfiguration("use_grippers")

    # ===== Paths =====
    xacro_path = PathJoinSubstitution([FindPackageShare(description_package), "urdf", description_file])
    controllers_yaml = PathJoinSubstitution([FindPackageShare(runtime_config_package), "config", controllers_file])
    rviz_config_file = PathJoinSubstitution([FindPackageShare(description_package), "rviz", "robot_description.rviz"])

    # ===== robot_description =====
    robot_description_cmd = Command([
        FindExecutable(name="xacro"), " ", xacro_path, " ",
        "mount_half_x:=",   mount_half_x, " ",
        "zero_pos:=",       zero_pos, " ",
        "left_zero_pos:=",  left_zero_pos, " ",
        "right_zero_pos:=", right_zero_pos, " ",
        "left_prefix:=left_ right_prefix:=right_ ",
        "can_device:=",     can_device, " ",
        "left_can_device:=",  left_can_device, " ",
        "right_can_device:=", right_can_device, " ",
        "can_fd:=",         can_fd, " ",
        "disable_torque:=", disable_torque, " ",
        "use_ros2_control:=", use_ros2_control, " ",
        "use_mock_hardware:=", use_mock_hardware, " ",
        "use_grippers:=", use_grippers,
    ])
    robot_description = {"robot_description": ParameterValue(robot_description_cmd, value_type=str)}

    # ===== Core Nodes =====
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="both",
        parameters=[robot_description, controllers_yaml],
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # RViz: 기본 ON (rviz:=true), 필요하면 rviz:=false로 끄기
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_file],
        output="log",
        condition=IfCondition(rviz),
    )

    # Mock 하드웨어일 때만 fake remap + 브리지 사용
    jsp_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen",
        parameters=[robot_description],
        remappings=[("/joint_states", "/fake_joint_states")],
        condition=IfCondition(PythonExpression([
            "'", use_mock_hardware, "' == 'true' and '", gui, "' == 'true'"
        ])),
    )

    # 실제 하드웨어일 때는 remap 없이 그대로 /joint_states 사용
    jsp_gui_node_hw = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui_hw",
        output="screen",
        parameters=[robot_description],
        condition=IfCondition(PythonExpression([
            "'", use_mock_hardware, "' == 'false' and '", gui, "' == 'true'"
        ])),
    )



    # ===== Spawner: JSB 먼저 =====
    jsb_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
        output="screen",
    )

    jsp_fpc_bridge_node = Node(
        package="openarm_static_bimanual_bringup",
        executable="jsp_fpc_bridge.py",
        name="jsp_fpc_bridge",
        output="screen",
        condition=IfCondition(PythonExpression([
            "'", use_mock_hardware, "' == 'true' and '", gui, "' == 'true'"
        ]))
    )


    # ===== active_mode 조건 (문자열 비교만) =====
    is_jtc    = IfCondition(PythonExpression(["'", active_mode, "' == 'jtc'"]))
    not_jtc   = IfCondition(PythonExpression(["'", active_mode, "' != 'jtc'"]))
    is_fpc    = IfCondition(PythonExpression(["'", active_mode, "' == 'fpc'"]))
    not_fpc   = IfCondition(PythonExpression(["'", active_mode, "' != 'fpc'"]))
    is_teleop = IfCondition(PythonExpression(["'", active_mode, "' == 'teleop'"]))
    not_tele  = IfCondition(PythonExpression(["'", active_mode, "' != 'teleop'"]))

    # ===== Controllers (JTC/FPC/Teleop) =====
    left_jtc_active = Node(
        package="controller_manager", executable="spawner",
        arguments=["left_joint_trajectory_controller", "-c", "/controller_manager"],
        output="screen", condition=is_jtc,
    )
    right_jtc_active = Node(
        package="controller_manager", executable="spawner",
        arguments=["right_joint_trajectory_controller", "-c", "/controller_manager"],
        output="screen", condition=is_jtc,
    )
    left_jtc_inactive = Node(
        package="controller_manager", executable="spawner",
        arguments=["left_joint_trajectory_controller", "-c", "/controller_manager", "--inactive"],
        output="screen", condition=not_jtc,
    )
    right_jtc_inactive = Node(
        package="controller_manager", executable="spawner",
        arguments=["right_joint_trajectory_controller", "-c", "/controller_manager", "--inactive"],
        output="screen", condition=not_jtc,
    )

    left_fpc_active = Node(
        package="controller_manager", executable="spawner",
        arguments=["left_forward_position_controller", "-c", "/controller_manager"],
        output="screen", condition=is_fpc,
    )
    right_fpc_active = Node(
        package="controller_manager", executable="spawner",
        arguments=["right_forward_position_controller", "-c", "/controller_manager"],
        output="screen", condition=is_fpc,
    )
    left_fpc_inactive = Node(
        package="controller_manager", executable="spawner",
        arguments=["left_forward_position_controller", "-c", "/controller_manager", "--inactive"],
        output="screen", condition=not_fpc,
    )
    right_fpc_inactive = Node(
        package="controller_manager", executable="spawner",
        arguments=["right_forward_position_controller", "-c", "/controller_manager", "--inactive"],
        output="screen", condition=not_fpc,
    )

    left_teleop_active = Node(
        package="controller_manager", executable="spawner",
        arguments=["left_teleop_stream_controller", "-c", "/controller_manager"],
        output="screen", condition=is_teleop,
    )
    right_teleop_active = Node(
        package="controller_manager", executable="spawner",
        arguments=["right_teleop_stream_controller", "-c", "/controller_manager"],
        output="screen", condition=is_teleop,
    )
    left_teleop_inactive = Node(
        package="controller_manager", executable="spawner",
        arguments=["left_teleop_stream_controller", "-c", "/controller_manager", "--inactive"],
        output="screen", condition=not_tele,
    )
    right_teleop_inactive = Node(
        package="controller_manager", executable="spawner",
        arguments=["right_teleop_stream_controller", "-c", "/controller_manager", "--inactive"],
        output="screen", condition=not_tele,
    )

    teleop_follower_node = Node(
        package="openarm_static_bimanual_bringup",
        executable="teleop_follower.py",
        name="teleop_follower",
        output="screen",
        parameters=[{
            "left_controller":  "left_teleop_stream_controller",
            "right_controller": "right_teleop_stream_controller",
        }],
        condition=IfCondition("false"),  # ← 파일 생길 때까지 절대 실행 안 함
    )


    gripper_left = Node(
        package="openarm_arduino_bridge", executable="servo_bridge",
        name="gripper_left_bridge", output="screen",
        parameters=[{
            "port": gripper_left_port, "baud": gripper_baud,
            "joint_name": "left_left_pris1",
            "js_topic": "/joint_states",
            "write_min_period_ms": 200,
            "inject_when_absent": True,
            "absent_timeout_ms": 300,
            "default_when_off": 0.0,
        }],
        condition=IfCondition(PythonExpression([
            "'", enable_gripper_left, "' == 'true' and '", use_mock_hardware, "' == 'false'"
        ])),
    )

    gripper_right = Node(
        package="openarm_arduino_bridge", executable="servo_bridge",
        name="gripper_right_bridge", output="screen",
        parameters=[{
            "port": gripper_right_port, "baud": gripper_baud,
            "joint_name": "right_left_pris1",
            "js_topic": "/joint_states",
            "write_min_period_ms": 200,
            "inject_when_absent": True,
            "absent_timeout_ms": 300,
            "default_when_off": 0.0,
        }],
        condition=IfCondition(PythonExpression([
            "'", enable_gripper_right, "' == 'true' and '", use_mock_hardware, "' == 'false'"
        ])),
    )


    # ===== Sequence: ros2_control → JSB → controllers (+ teleop) =====
    delay_jsb_after_ctrl = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=control_node,
            on_start=[TimerAction(period=2.0, actions=[jsb_spawner])]
        )
    )

    spawn_after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=jsb_spawner,
            on_exit=[TimerAction(period=0.5, actions=[
                # JTC
                left_jtc_active, right_jtc_active, left_jtc_inactive, right_jtc_inactive,
                # FPC
                left_fpc_active, right_fpc_active, left_fpc_inactive, right_fpc_inactive,
                # Teleop
                left_teleop_active, right_teleop_active, left_teleop_inactive, right_teleop_inactive,
                # teleop follower (조건식으로 필요한 때만 실행)
                teleop_follower_node,
            ])],
        )
    )

    return LaunchDescription(
        declared_arguments + [
            control_node,
            robot_state_pub_node,
            rviz_node,

            # GUI (모크/실제 분리)
            jsp_gui_node,
            jsp_gui_node_hw,
            jsp_fpc_bridge_node,

            # Grippers (조건식 적용됨 → 필요할 때만 실행)
            gripper_left,
            gripper_right,

            delay_jsb_after_ctrl,
            spawn_after_jsb,
        ]
    )

