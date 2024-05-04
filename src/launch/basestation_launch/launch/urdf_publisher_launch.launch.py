from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

import yaml

launch_file_path = os.path.split(os.path.realpath(__file__))[0] + '/'
related_param_path = os.path.join(launch_file_path, '../urdf/urdf_related_param.yaml')

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
    pkg_name = 'basestation_launch'
    urdf_file = 'f1tenth_ucsd.urdf'
    desc_dir = get_package_share_directory(pkg_name)

    related_param_num, related_param_str = yaml_decode(related_param_path)
    # print(related_param_num, '\n', related_param_str)

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

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        arguments=[urdf_file_path],
        # condition=UnlessCondition(LaunchConfiguration('gui'))
    )

    ## tf2 - lidar_link to laser frame
    node_tf2_fp2laser = Node(
        name='tf2_ros_fp_laser',
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=[str(related_param_num['x']),
                   str(related_param_num['y']),
                   str(related_param_num['z']),
                   str(related_param_num['yaw']),
                   str(related_param_num['pitch']),
                   str(related_param_num['roll']),
                   related_param_str['frame_id'],
                   related_param_str['child_frame_id']],   
    )
    
    ld = LaunchDescription()
    ld.add_action(robot_state_publisher_node)
    ld.add_action(joint_state_publisher_node)
    ld.add_action(node_tf2_fp2laser)

    return ld
