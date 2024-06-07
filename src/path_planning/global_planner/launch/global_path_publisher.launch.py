from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

import yaml

def yaml_decode(yaml_input_path):
        with open(yaml_input_path, "r") as file:
            yaml_inputs = yaml.load(file, Loader=yaml.FullLoader)
            numeric_inputs = {}
            string_inputs = {}
            for key in yaml_inputs:
                value = yaml_inputs[key]
                if isinstance(value, float) or isinstance(value, int):
                    numeric_inputs[key] = value
                elif isinstance(value, str):
                    string_inputs[key] = value
            return numeric_inputs, string_inputs

def generate_launch_description():
    pkg_name = 'global_planner'

    map_save_path_pkg = 'basestation_launch'
    map_select_file = 'map_select.yaml'

    map_select_yaml_path = os.path.join(
        get_package_share_directory(map_save_path_pkg),
        'param',
        map_select_file
    )

    map_info_num, map_info_str = yaml_decode(map_select_yaml_path)
    # print('map_info_num', '\n', 'map_info_str')

    map_file_dir = os.path.join(
        map_info_str['absolute_path_prefix'], 
        map_info_str['default_path_prefix'],
        map_info_str['custom_prefix']
    )

    map_dir_launch_arg = DeclareLaunchArgument('map_dir', default_value=map_file_dir)

    map_arguments = [map_dir_launch_arg]

    override_params = {
         'map_path': LaunchConfiguration('map_dir'),
    }

    global_republisher = Node(
        package='global_planner',
        executable='global_trajectory_publisher',
        name='global_republisher',
        output='screen',
        parameters=[override_params]
    )

    ld = LaunchDescription([])
    for arg in map_arguments:
        ld.add_entity(arg)
    ld.add_action(global_republisher)

    return ld