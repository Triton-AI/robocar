import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
import yaml


def generate_launch_description():
    pkg_name = 'basestation_launch'
    pkg_dir = '/home/jetson/projects/robocar/src/launch/basestation_launch/rviz'
    config_file = 'sensor_visualization.rviz'
    
    config = os.path.join(
        get_package_share_directory(pkg_name),
        'rviz',
        config_file)

    ld = LaunchDescription()

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', [config]])

    ld.add_action(rviz2_node)
    return ld
