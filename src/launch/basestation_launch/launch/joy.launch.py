from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    joy_config = os.path.join(
        get_package_share_directory('basestation_launch'),
        'param',
        'joy.param.yaml'
    )
    joy_teleop_config = os.path.join(
        get_package_share_directory('basestation_launch'),
        'param',
        'joy_teleop.param.yaml'
    )

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
        parameters=[joy_config]
    )

    ld.add_action(joy_node)
    ld.add_action(joy_teleop_node)
    return ld
