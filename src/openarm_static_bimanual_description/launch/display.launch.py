from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.conditions import IfCondition

def generate_launch_description():
    pkg_share = FindPackageShare("openarm_static_bimanual_description")
    pkg_path  = Path(pkg_share.find("openarm_static_bimanual_description"))

    default_model      = str(pkg_path / "urdf/openarm_static_bimanual.urdf.xacro")
    default_rvizconfig = str(pkg_path / "rviz/openarm_bimanual.rviz")

    # 공통/표시
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_jsb      = LaunchConfiguration("use_jsb")   # SIM일 때만 JSB 사용
    gui          = LaunchConfiguration("gui")       # JSB를 켤 때 GUI 쓸지

    model      = LaunchConfiguration("model")
    rvizconfig = LaunchConfiguration("rvizconfig")

    # 단팔 UX 대응 인자들
    zero_pos       = LaunchConfiguration("zero_pos")
    left_zero_pos  = LaunchConfiguration("left_zero_pos")
    right_zero_pos = LaunchConfiguration("right_zero_pos")

    prefix         = LaunchConfiguration("prefix")
    left_prefix    = LaunchConfiguration("left_prefix")
    right_prefix   = LaunchConfiguration("right_prefix")

    can_device       = LaunchConfiguration("can_device")
    left_can_device  = LaunchConfiguration("left_can_device")
    right_can_device = LaunchConfiguration("right_can_device")

    can_fd           = LaunchConfiguration("can_fd")
    disable_torque   = LaunchConfiguration("disable_torque")
    use_ros2_control = LaunchConfiguration("use_ros2_control")
    mount_half_x     = LaunchConfiguration("mount_half_x")

    description_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_share, "launch", "description.launch.py"])
        ]),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "model": model,
            "zero_pos": zero_pos,
            "left_zero_pos": left_zero_pos,
            "right_zero_pos": right_zero_pos,
            "prefix": prefix,
            "left_prefix": left_prefix,
            "right_prefix": right_prefix,
            "can_device": can_device,
            "left_can_device": left_can_device,
            "right_can_device": right_can_device,
            "can_fd": can_fd,
            "disable_torque": disable_torque,
            "use_ros2_control": use_ros2_control,
            "mount_half_x": mount_half_x,
        }.items(),
    )

    # ---- 조건식: PythonExpression 또는 AndSubstitution/NotSubstitution 사용 ----
    jsb_text_cond = IfCondition(
        PythonExpression(["'", use_jsb, "' == 'true' and '", gui, "' == 'false'"])
    )
    jsb_gui_cond  = IfCondition(
        PythonExpression(["'", use_jsb, "' == 'true' and '", gui, "' == 'true'"])
    )

    jsp_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        condition=jsb_text_cond,
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    jsp_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        condition=jsb_gui_cond,
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rvizconfig],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    return LaunchDescription([
        # 표시/경로
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("use_jsb",      default_value="false"),  # HW 기본 끔
        DeclareLaunchArgument("gui",          default_value="true"),   # JSB 켤 때만 의미
        DeclareLaunchArgument("model",        default_value=default_model),
        DeclareLaunchArgument("rvizconfig",   default_value=default_rvizconfig),

        # 단팔 UX 대응
        DeclareLaunchArgument("zero_pos",       default_value="up"),
        DeclareLaunchArgument("left_zero_pos",  default_value=""),
        DeclareLaunchArgument("right_zero_pos", default_value=""),

        DeclareLaunchArgument("prefix",       default_value=""),
        DeclareLaunchArgument("left_prefix",  default_value="left_"),
        DeclareLaunchArgument("right_prefix", default_value="right_"),

        DeclareLaunchArgument("can_device",       default_value="can0"),
        DeclareLaunchArgument("left_can_device",  default_value=""),
        DeclareLaunchArgument("right_can_device", default_value=""),

        # HW/간격
        DeclareLaunchArgument("can_fd",           default_value="false"),
        DeclareLaunchArgument("disable_torque",   default_value="true"),
        DeclareLaunchArgument("use_ros2_control", default_value="true"),
        DeclareLaunchArgument("mount_half_x",     default_value="0.30"),

        description_include,
        jsp_node,
        jsp_gui_node,
        rviz_node,
    ])
