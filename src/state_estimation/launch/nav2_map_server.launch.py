from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
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
    pkg_name = 'state_estimation'
    desc_dir = get_package_share_directory(pkg_name)

    map_save_path_pkg = 'basestation_launch'
    map_select_file = 'map_select.yaml'
    
    map_select_yaml_path = os.path.join(
        get_package_share_directory(map_save_path_pkg),
        'param',
        map_select_file
    )

    map_info_num, map_info_str = yaml_decode(map_select_yaml_path)
    # print('map_info_num', '\n', 'map_info_str')

    map_file = os.path.join(
        get_package_share_directory(map_save_path_pkg), 
        map_info_str['default_path_prefix'],
        map_info_str['custom_prefix'], 
        map_info_str['map_yaml_file']   # where your map *.yaml is
    ) 
    
    ## map_server
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'use_sim_time': False}, 
                    {'topic_name': "map"},
                    {'frame_id': "map"},    # make sure the frame exist to visualize the OccupancyGrid
                    {'yaml_filename':map_file}]
    )

    ## lifecycle_manager - brinup map_server lifecycle
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_mapper',
        output='screen',
        parameters=[{'use_sim_time': False},
                    {'autostart': True},
                    {'node_names': ['map_server']}]
    )

    ld = LaunchDescription([])
    ld.add_action(map_server_node)
    ld.add_action(lifecycle_manager_node)

    return ld