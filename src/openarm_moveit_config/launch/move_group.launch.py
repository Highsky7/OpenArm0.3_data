# SPDX-License-Identifier: Apache-2.0
import os
import yaml
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def load_file(pkg: str, rel_path: str) -> str:
    path = os.path.join(get_package_share_directory(pkg), rel_path)
    with open(path, 'r') as f:
        return f.read()

def load_yaml(pkg: str, rel_path: str) -> dict:
    path = os.path.join(get_package_share_directory(pkg), rel_path)
    with open(path, 'r') as f:
        return yaml.safe_load(f)

def generate_launch_description():
    desc_pkg = 'openarm_description'
    cfg_pkg  = 'openarm_moveit_config'

    # ---- xacro을 Python에서 직접 파싱해서 robot_description 생성 ----
    xacro_path = os.path.join(
        get_package_share_directory(desc_pkg), 'urdf', 'openarm.urdf.xacro'
    )
    xacro_doc = xacro.process_file(xacro_path)  # 필요시 매크로 인자: xacro.process_file(xacro_path, mappings={'side': 'right'})
    robot_description = {'robot_description': xacro_doc.toxml()}

    # SRDF / Kinematics / OMPL / Controllers
    robot_description_semantic   = {'robot_description_semantic': load_file(cfg_pkg, 'srdf/openarm.srdf')}
    robot_description_kinematics = {'robot_description_kinematics': load_yaml(cfg_pkg, 'config/kinematics.yaml')}
    ompl_planning_yaml           = load_yaml(cfg_pkg, 'config/ompl_planning.yaml')
    moveit_controllers_yaml      = load_yaml(cfg_pkg, 'config/moveit_controllers.yaml')

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description],
        output='screen',
    )

    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0','0','0','0','0','0','world','base_link'],
        output='screen',
    )

    move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            ompl_planning_yaml,
            moveit_controllers_yaml,
            {'planning_pipelines': ['ompl']},
        ],
    )
    ik_params = os.path.join(
    get_package_share_directory('openarm_moveit_config'),
    'config', 'ik_exec_params.yaml'
    )

    ik_exec = Node(
    package='openarm_moveit_config',
    executable='openarm_ik_exec',
    output='screen',
    parameters=[ik_params]   # ← YAML 경로
    )


    return LaunchDescription([rsp, static_tf, move_group, ik_exec])
