from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    teleop_serial_node = Node(
        package="openarm_bringup",
        executable="teleop_serial_publisher.py",
        name="teleop_serial_publisher",
        output="screen",
        parameters=[{
            "port": "/dev/ttyACM0",   # 필요시 override
            "baud": 115200,
            "names": ["tele0","tele1","tele2","tele3","tele4","tele5","tele6","tele7"]
        }],
    )
    return LaunchDescription([teleop_serial_node])