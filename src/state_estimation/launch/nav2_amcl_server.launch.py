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

    nav2_amcl_yaml = os.path.join(
        desc_dir, 
        'param', 
        'amcl_config.yaml'
    )
    
    ## amcl_server
    nav2_amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[nav2_amcl_yaml]
    )

    ## lifecycle_manager - brinup amcl_server lifecycle
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_mapper',
        output='screen',
        parameters=[{'use_sim_time': False},
                    {'autostart': True},
                    {'node_names': ['amcl']}]
    )

    ld = LaunchDescription([])
    ld.add_action(nav2_amcl_node)
    ld.add_action(lifecycle_manager_node)

    return ld