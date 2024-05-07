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
    pkg_name = 'planner'
    desc_dir = get_package_share_directory(pkg_name)

    controller_yaml = os.path.join(desc_dir, 'param', 'controller.yaml')
    bt_navigator_yaml = os.path.join(desc_dir, 'param', 'bt_navigator.yaml')
    planner_yaml = os.path.join(desc_dir, 'param', 'planner_server.yaml')
    recovery_yaml = os.path.join(desc_dir, 'param', 'recovery.yaml')

    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[controller_yaml]
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[planner_yaml]
    )

    # recoveries_server is in nav2_recoveries in ros2 foxy
    recovery_server = Node(
        package='nav2_recoveries',      # nav2_behavior (for newer distro of ROS 2)
        executable='recoveries_server', # behavior_server (for newer distro of ROS 2)
        name='recoveries_server',
        output='screen',
        parameters=[recovery_yaml]
    )

    bt_nav_server = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[bt_navigator_yaml]
    )

    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_pathplanner',
        output='screen',
        parameters=[{'autostart': True},
                    {'node_names': ['planner_server',
                                    'controller_server',
                                    'recoveries_server',
                                    'bt_navigator']}]
    )
    
    ld = LaunchDescription([])
    ld.add_action(controller_server)
    ld.add_action(planner_server)
    ld.add_action(recovery_server)
    ld.add_action(bt_nav_server)
    ld.add_action(lifecycle_manager_node)

    return ld