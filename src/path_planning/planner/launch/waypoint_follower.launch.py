from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_name = 'planner'
    desc_dir = get_package_share_directory(pkg_name)

    waypoint_follower_yaml = os.path.join(desc_dir, 'param', 'waypoint_follower.yaml')

    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[waypoint_follower_yaml]
    )

    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_pathplanner',
        output='screen',
        parameters=[{'autostart': True},
                    {'node_names': ['waypoint_follower']}]
    )
    
    ld = LaunchDescription([])
    ld.add_action(waypoint_follower)
    ld.add_action(lifecycle_manager_node)

    return ld