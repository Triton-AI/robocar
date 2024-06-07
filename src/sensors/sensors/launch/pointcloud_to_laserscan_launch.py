from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

import yaml

launch_file_path = os.path.split(os.path.realpath(__file__))[0] + '/'
slice_param_path = os.path.join(launch_file_path, '../param/slice_frame_info.yaml')

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
    sensors_pkg = 'sensors'

    slice_info_num, slice_info_str = yaml_decode(slice_param_path)
    # print(slice_info_num, '\n', slice_info_str)
    
    pcl_to_scan_config = os.path.join(
        get_package_share_directory(sensors_pkg),
        'param',
        'pcl_to_scan.yaml'
    )

    return LaunchDescription([
        # DeclareLaunchArgument(
        #     name='scanner', default_value='scanner',
        #     description='Namespace for sample topics'
        # ),
        # Node(
        #     package='pointcloud_to_laserscan', executable='dummy_pointcloud_publisher',
        #     remappings=[('cloud', [LaunchConfiguration(variable_name='scanner'), '/cloud'])],
        #     parameters=[{'cloud_frame_id': 'cloud', 'cloud_extent': 2.0, 'cloud_size': 500}],
        #     name='cloud_publisher'
        # ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            arguments=[str(slice_info_num['x']),
                       str(slice_info_num['y']),
                       str(slice_info_num['z']),
                       str(slice_info_num['yaw']),
                       str(slice_info_num['pitch']),
                       str(slice_info_num['roll']),
                       slice_info_str['frame_id'],
                       slice_info_str['child_frame_id']]
        ),
        Node(
            package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
            remappings=[('cloud_in', slice_info_str['pointcloud_topic']),
                        # ('scan', [LaunchConfiguration(variable_name='scanner'), slice_info_str['slice_scan_topic']]),
                        ('scan', slice_info_str['slice_scan_topic'])],
            parameters=[pcl_to_scan_config],
            name='pointcloud_to_laserscan'
        )
    ])
