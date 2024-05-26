from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

import yaml

def read_yaml(yaml_input_path):
    with open(yaml_input_path, "r") as file:
        yaml_inputs = yaml.load(file, Loader=yaml.SafeLoader)
        return yaml_inputs

def parse_select(select_file_path: str, kalman_filter_type: str):
    """
    parse_select(select_file_path, kalman_filter_type)
        - The function helps parse the select file 
          for `ekf_node` or `ukf_node` in `robot_localization` pkg.
        - Please name your select yaml file as `ekf_select.yaml` or `ukf_select.yaml`.
        - Specify `ekf_config` or `ukf_config` in the 1st line of your yaml file.
    """
    select_prefix = kalman_filter_type + '_select'
    override_prefix = 'override_' + kalman_filter_type + '_config'
    select_yaml = read_yaml(select_file_path)
    arguments_ = []
    override_params_ = dict()
    if select_yaml[select_prefix][override_prefix]:
        num_odoms = 0
        num_imus = 0
        odom_names = []
        imu_names = []
        for odom_arg, odom_dict in select_yaml[select_prefix]['odom'].items():
            if odom_dict['enable'] == 1:
                odom_arg_num = 'odom' + str(num_odoms)
                arguments_.append(DeclareLaunchArgument(str(odom_arg),
                                                        default_value=odom_dict['odom_topic']))
                override_params_[odom_arg_num] = LaunchConfiguration(str(odom_arg))

                odom_arg_config = odom_arg_num + '_config'
                arguments_.append(DeclareLaunchArgument(str(odom_arg + '_config'),
                                                        default_value=
                                                        str(odom_dict['odom_config'])))
                override_params_[odom_arg_config] = LaunchConfiguration(str(odom_arg + '_config'))
                
                odom_arg_queue_size = odom_arg_num + '_queue_size'
                arguments_.append(DeclareLaunchArgument(str(odom_arg + '_queue_size'),
                                                        default_value=
                                                        str(odom_dict['odom_queue_size'])))
                override_params_[odom_arg_queue_size] = LaunchConfiguration(str(odom_arg + '_queue_size'))

                odom_arg_rel = odom_arg_num + '_relative'
                arguments_.append(DeclareLaunchArgument(str(odom_arg + '_relative'),
                                                        default_value=
                                                        str(odom_dict['odom_relative'])))
                override_params_[odom_arg_rel] = LaunchConfiguration(str(odom_arg + '_relative'))

                num_odoms += 1
                odom_names.append(str(odom_arg))

        for imu_arg, imu_dict in select_yaml[select_prefix]['imu'].items():
            if imu_dict['enable'] == 1:
                imu_arg_num = 'imu' + str(num_imus)
                arguments_.append(DeclareLaunchArgument(str(imu_arg),
                                                        default_value=imu_dict['imu_topic']))
                override_params_[imu_arg_num] = LaunchConfiguration(str(imu_arg))

                imu_arg_config = imu_arg_num + '_config'
                arguments_.append(DeclareLaunchArgument(str(imu_arg + '_config'),
                                                        default_value=
                                                        str(imu_dict['imu_config'])))
                override_params_[imu_arg_config] = LaunchConfiguration(str(imu_arg + '_config'))

                imu_arg_queue_size = imu_arg_num + '_queue_size'
                arguments_.append(DeclareLaunchArgument(str(imu_arg + '_queue_size'),
                                                        default_value=
                                                        str(imu_dict['imu_queue_size'])))
                override_params_[imu_arg_queue_size] = LaunchConfiguration(str(imu_arg + '_queue_size'))

                imu_arg_rel = imu_arg_num + '_relative'
                arguments_.append(DeclareLaunchArgument(str(imu_arg + '_relative'),
                                                        default_value=
                                                        str(imu_dict['imu_relative'])))
                override_params_[imu_arg_rel] = LaunchConfiguration(str(imu_arg + '_relative'))

                imu_arg_gravity = imu_arg_num + '_remove_gravitational_acceleration'
                arguments_.append(DeclareLaunchArgument(str(imu_arg + '_gravity'),
                                                        default_value=
                                                        str(imu_dict['remove_gravitational_acceleration'])))
                override_params_[imu_arg_gravity] = LaunchConfiguration(str(imu_arg + '_gravity'))
                
                num_imus += 1
                imu_names.append(str(imu_arg))

        print('[Sensor Fusion]', 'Num of Odom:', num_odoms, ', Names of Odom:', odom_names)
        print('[Sensor Fusion]', 'Num of IMU:', num_imus, ', Names of IMU:', imu_names)
    return arguments_, override_params_

def generate_launch_description():
    pkg_name = 'state_estimation'
    config_file = 'ekf_config.yaml'
    select_file = 'ekf_select.yaml'
    desc_dir = get_package_share_directory(pkg_name)

    config_file_path = os.path.join(
        desc_dir,
        'param',
        config_file
    )

    select_file_path = os.path.join(
        desc_dir,
        'param',
        select_file
    )

    ekf_arguments, override_params = parse_select(select_file_path, 'ekf')

    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            config_file_path, 
            {'use_sim_time': False},
            override_params
        ]
    )

    ld = LaunchDescription()
    for arg in ekf_arguments:
        ld.add_entity(arg)
    ld.add_action(robot_localization_node)

    return ld
