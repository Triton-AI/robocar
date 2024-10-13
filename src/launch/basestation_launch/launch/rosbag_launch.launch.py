import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
import yaml


        # cmd=['ros2', 'bag', 'record', '-o', 'sven_cbf2', '/teleop', '/scan', '/imu_topic', '/odom', '/slam_out_pose'],

def generate_launch_description():
    ld = LaunchDescription()
    rosbag_execute = ExecuteProcess(
        name='execute_ros_bag',
        cmd=['ros2', 'bag', 'record', '-o', 'icra_curve', '-a'],
        output='screen',
        shell='True'
        )
    ld.add_action(rosbag_execute)
    return ld
