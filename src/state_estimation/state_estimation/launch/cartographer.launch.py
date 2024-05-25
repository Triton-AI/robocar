from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_name = 'state_estimation'
    
    desc_dir = get_package_share_directory(pkg_name)
    cartographer_config_dir = os.path.join(desc_dir, 'param')
    configuration_basename = 'cartographer.lua'

    cartographer_ros_node = Node(
        package='cartographer_ros', 
        executable='cartographer_node', 
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': False}],
        arguments=['-configuration_directory', cartographer_config_dir,
                    '-configuration_basename', configuration_basename]
    )

    occupancy_grid_node =Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        output='screen',
        name='occupancy_grid_node',
        parameters=[{'use_sim_time': False}],
        arguments=['-resolution', '0.05', '-publish_period_sec', '1.0']
    )

    ld = LaunchDescription([])
    ld.add_action(cartographer_ros_node)
    ld.add_action(occupancy_grid_node)

    return ld