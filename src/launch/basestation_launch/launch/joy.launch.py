from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    print(get_package_share_directory('joy_teleop'))
    ctrl_pkg = 'basestation_launch'
    joy_config = os.path.join(
        get_package_share_directory(ctrl_pkg),
        'param',
        'joy.param.yaml'
    )
    joy_teleop_config = os.path.join(
        get_package_share_directory(ctrl_pkg),
        'param',
        'joy_teleop.param.yaml'
    )
    # joy_la = DeclareLaunchArgument(
    #     'joy_config',
    #     default_value=joy_teleop_config,
    #     description='Descriptions for joy and joy_teleop configs')

    ld = LaunchDescription([])

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy',
        parameters=[joy_config]
    )

    joy_teleop_node = Node(
        package='joy_teleop',
        executable='joy_teleop',
        name='joy_teleop',
        parameters=[joy_teleop_config]
    )

    ld.add_action(joy_node)
    ld.add_action(joy_teleop_node)
    return ld
