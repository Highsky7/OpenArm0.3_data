from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_share = FindPackageShare("openarm_static_bimanual_description")
    pkg_path  = Path(pkg_share.find("openarm_static_bimanual_description"))
    default_model = str(pkg_path / "urdf/openarm_static_bimanual.urdf.xacro")

    # ---- Launch args
    use_sim_time     = LaunchConfiguration("use_sim_time")
    model            = LaunchConfiguration("model")

    # zero_pos: 공통/개별
    zero_pos         = LaunchConfiguration("zero_pos")
    left_zero_pos    = LaunchConfiguration("left_zero_pos")
    right_zero_pos   = LaunchConfiguration("right_zero_pos")

    # prefix: 공통/개별
    prefix           = LaunchConfiguration("prefix")
    left_prefix      = LaunchConfiguration("left_prefix")
    right_prefix     = LaunchConfiguration("right_prefix")

    # CAN: 공통/개별
    can_device       = LaunchConfiguration("can_device")
    left_can_device  = LaunchConfiguration("left_can_device")
    right_can_device = LaunchConfiguration("right_can_device")

    # HW 옵션
    can_fd           = LaunchConfiguration("can_fd")
    disable_torque   = LaunchConfiguration("disable_torque")
    use_ros2_control = LaunchConfiguration("use_ros2_control")

    # 장착 간격 (URDF는 X축 기준)
    mount_half_x     = LaunchConfiguration("mount_half_x")

    # ---- xacro command
    robot_description_cmd = Command([
        "xacro ", model,
        " mount_half_x:=",     mount_half_x,
        " zero_pos:=",         zero_pos,           # 공통
        " left_zero_pos:=",    left_zero_pos,      # 개별(우선)
        " right_zero_pos:=",   right_zero_pos,     # 개별(우선)
        " prefix:=",           prefix,             # 공통
        " left_prefix:=",      left_prefix,        # 개별(우선)
        " right_prefix:=",     right_prefix,       # 개별(우선)
        " can_device:=",       can_device,         # 공통
        " left_can_device:=",  left_can_device,    # 개별(우선)
        " right_can_device:=", right_can_device,   # 개별(우선)
        " can_fd:=",           can_fd,
        " disable_torque:=",   disable_torque,
        " use_ros2_control:=", use_ros2_control,
    ])

    return LaunchDescription([
        # 공통
        DeclareLaunchArgument("use_sim_time",   default_value="false"),
        DeclareLaunchArgument("model",          default_value=default_model),

        # 단팔 UX 대응 기본값
        DeclareLaunchArgument("zero_pos",       default_value="up"),
        DeclareLaunchArgument("prefix",         default_value=""),     # 공통 prefix
        DeclareLaunchArgument("can_device",     default_value="can0"),

        # 개별 세부 조정(미지정 시 xacro에서 공통값 사용)
        DeclareLaunchArgument("left_zero_pos",   default_value=""),
        DeclareLaunchArgument("right_zero_pos",  default_value=""),    # ← 오타 수정
        DeclareLaunchArgument("left_prefix",     default_value="left_"),
        DeclareLaunchArgument("right_prefix",    default_value="right_"),
        DeclareLaunchArgument("left_can_device", default_value=""),
        DeclareLaunchArgument("right_can_device", default_value=""),

        # HW 옵션 & 간격
        DeclareLaunchArgument("can_fd",           default_value="false"),
        DeclareLaunchArgument("disable_torque",   default_value="true"),
        DeclareLaunchArgument("use_ros2_control", default_value="true"),
        DeclareLaunchArgument("mount_half_x",     default_value="0.30"),  # ← X로 통일

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[{
                "robot_description": ParameterValue(robot_description_cmd, value_type=str),
                "use_sim_time": use_sim_time,
            }],
            output="screen",
        ),
    ])
