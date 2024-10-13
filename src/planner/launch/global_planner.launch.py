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
    main_stack_pkg = 'basestation_launch'
    pkg_name = 'global_planner'
    desc_dir = get_package_share_directory(main_stack_pkg)

    params_yaml = os.path.join(desc_dir, 'param', 'global_planner', 'global_planner_params.yaml')
    
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

    map_name = map_info_str['map_yaml_file'][:map_info_str['map_yaml_file'].index('.')]
    
    map_yaml_info_num, map_yaml_info_str = yaml_decode(os.path.join(map_file_dir, map_info_str['map_yaml_file']))
    map_type = map_yaml_info_str['image'][map_yaml_info_str['image'].index('.')+1:]

    map_name_launch_arg = DeclareLaunchArgument('map_name', default_value=map_name)
    map_dir_launch_arg = DeclareLaunchArgument('map_dir', default_value=map_file_dir)
    map_type_launch_arg = DeclareLaunchArgument('map_type', default_value=map_type)

    map_arguments = [map_name_launch_arg, map_dir_launch_arg, map_type_launch_arg]

    override_params = {
         'map_name': LaunchConfiguration('map_name'),
         'map_dir': LaunchConfiguration('map_dir'),
         'map_type': LaunchConfiguration('map_type')
    }

    global_planner = Node(
        package='global_planner',
        executable='global_planner',
        # namespace='/',
        name='global_planner',
        output='screen',
        parameters=[params_yaml, override_params]
    )

    global_republisher = Node(
        package='global_planner',
        executable='global_trajectory_publisher',
        name='global_republisher',
        output='screen',
        parameters=[]
    )

    ld = LaunchDescription([])
    for arg in map_arguments:
        ld.add_entity(arg)
    ld.add_action(global_planner)
    ld.add_action(global_republisher)

    return ld