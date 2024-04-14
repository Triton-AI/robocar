from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_name = 'basestation_launch'
    urdf_file = 'f1tenth_livox.urdf'
    desc_dir = get_package_share_directory(pkg_name)

    urdf_file_path = os.path.join(
        desc_dir,
        'urdf',
        urdf_file
    )

    with open(urdf_file_path, 'r') as infile:
        urdf_content = infile.read()

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_node',
        output='screen',
        parameters=[
            {'robot_description':urdf_content,
            'use_sim_time': False}
        ]
    )

    ## tf2 - lidar_link to laser frame
    node_tf2_fp2laser = Node(
        name='tf2_ros_fp_laser',
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=['0', '0', '0', '0.0', '0.0', '0.0', 'lidar_link', 'livox_frame'],   
    )
    
    ld = LaunchDescription()
    ld.add_action(robot_state_publisher_node)
    ld.add_action(node_tf2_fp2laser)

    return ld
