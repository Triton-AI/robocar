from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    some_pkg = 'twist_to_ackermann'
    param_file = 'twist_to_ackermann.yaml'
    some_node = 'twist_to_ackermann_node'
    desc_dir = get_package_share_directory(some_pkg)

    param_file_path = os.path.join(
        desc_dir,
        'param',
        param_file
    )

    tta_node = Node(
        package=some_pkg,
        executable=some_node,
        output='screen',
        parameters=[param_file_path]
    )
    
    ld = LaunchDescription()
    ld.add_action(tta_node)

    return ld
