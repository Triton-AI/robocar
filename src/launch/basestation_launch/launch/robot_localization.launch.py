from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_name = 'basestation_launch'
    config_file = 'ekf.yaml'
    desc_dir = get_package_share_directory(pkg_name)

    config_file_path = os.path.join(
        desc_dir,
        'config',
        config_file
    )

    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            config_file_path, {
            'use_sim_time': False}
        ]
    )

    ld = LaunchDescription()
    ld.add_action(robot_localization_node)

    return ld
