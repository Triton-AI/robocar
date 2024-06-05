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
    config_file = 'global_wpnts_detection.yaml'
    config_file_yaml = os.path.join(desc_dir, 'param', config_file)

    waypoint_detection = Node(
        package=pkg_name,
        executable='waypoint_detection',
        name='waypoint_follower',
        output='screen',
        parameters=[config_file_yaml],
        arguments=['--ros-args', '--disable-stdout-logs'],
    )
    
    ld = LaunchDescription([])
    ld.add_action(waypoint_detection)

    return ld