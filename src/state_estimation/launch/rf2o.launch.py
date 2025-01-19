import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'state_estimation'
    config_file = 'rf2o_config.yaml'
    desc_dir = get_package_share_directory(pkg_name)

    config_file_path = os.path.join(
        desc_dir,
        'param',
        config_file
    )

    rf2o_node = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        output='screen',
        parameters=[config_file_path],
        arguments=['--ros-args', '--disable-stdout-logs'],
    )

    ld = LaunchDescription()
    ld.add_action(rf2o_node)

    return ld
