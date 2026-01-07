from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from datetime import datetime

def generate_launch_description():
    # Generate a unique filename with timestamp
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    default_filename = TextSubstitution(text=f'recorded_waypoints_{timestamp}.json')

    return LaunchDescription([
        DeclareLaunchArgument(
            'waypoint_file',
            default_value=default_filename,
            description='Name of the JSON file to save recorded waypoints (e.g., my_waypoints.json)'
        ),
        Node(
            package='openarm_recorder',
            executable='recorder_node',
            name='joint_state_recorder',
            output='screen',
            emulate_tty=True, # Required for keyboard input
            parameters=[
                {'waypoint_filename': LaunchConfiguration('waypoint_file')}
            ]
        )
    ])
