from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'waypoint_file',
            default_value='recorded_waypoints.json',
            description='Name of the JSON file containing recorded waypoints (e.g., my_waypoints.json)'
        ),
        DeclareLaunchArgument(
            'left_joint_names',
            default_value=['left_rev1', 'left_rev2', 'left_rev3', 'left_rev4', 'left_rev5', 'left_rev6', 'left_rev7'],
            description='List of canonical joint names for the left arm'
        ),
        DeclareLaunchArgument(
            'right_joint_names',
            default_value=['right_rev1', 'right_rev2', 'right_rev3', 'right_rev4', 'right_rev5', 'right_rev6', 'right_rev7'],
            description='List of canonical joint names for the right arm'
        ),
        Node(
            package='openarm_player',
            executable='player_node',
            name='joint_trajectory_player',
            output='screen',
            parameters=[
                {'waypoint_file': LaunchConfiguration('waypoint_file')},
                {'left_joint_names': ['left_rev1', 'left_rev2', 'left_rev3', 'left_rev4', 'left_rev5', 'left_rev6', 'left_rev7']},
                {'right_joint_names': ['right_rev1', 'right_rev2', 'right_rev3', 'right_rev4', 'right_rev5', 'right_rev6', 'right_rev7']}
            ]
        )
    ])
