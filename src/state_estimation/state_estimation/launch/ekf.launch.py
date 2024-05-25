from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

import yaml

def read_yaml(yaml_input_path):
    with open(yaml_input_path, "r") as file:
        yaml_inputs = yaml.load(file, Loader=yaml.SafeLoader)
        return yaml_inputs

def generate_launch_description():
    pkg_name = 'state_estimation'
    config_file = 'ekf_config.yaml'
    select_file = 'ekf_select.yaml'
    desc_dir = get_package_share_directory(pkg_name)

    config_file_path = os.path.join(
        desc_dir,
        'param',
        config_file
    )

    select_file_path = os.path.join(
        desc_dir,
        'param',
        select_file
    )

    select_yaml = read_yaml(select_file_path)
    # if select_yaml['ekf_select']['override_ekf_config'] == 

    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            config_file_path, 
            {'use_sim_time': False}
        ]
    )

    ld = LaunchDescription()
    ld.add_action(robot_localization_node)

    return ld
