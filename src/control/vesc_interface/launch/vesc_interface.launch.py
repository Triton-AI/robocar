from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
from base_common import get_share_file

def generate_launch_description():
    launch_dir = 'vesc_interface'

    ackermann_vesc_param = get_share_file(launch_dir, 'param', 'ackermann_vesc.param.yaml')
    vesc_odom_param = get_share_file(launch_dir, 'param', 'vesc_odom.param.yaml')
    vesc_driver_param = get_share_file(launch_dir, 'param', 'vesc_driver.param.yaml')
    mux_param = get_share_file(launch_dir, 'param', 'ackermann_mux.param.yaml')

    ackermann_to_vesc_node = Node(
        package='vesc_ackermann',
        executable='ackermann_to_vesc_node',
        name='ackermann_to_vesc_node',
        parameters=[ackermann_vesc_param]
    )
    vesc_to_odom_node = Node(
        package='vesc_ackermann',
        executable='vesc_to_odom_node',
        name='vesc_to_odom_node',
        parameters=[vesc_odom_param]
    )
    vesc_driver_node = Node(
        package='vesc_driver',
        executable='vesc_driver_node',
        name='vesc_driver_node',
        parameters=[vesc_driver_param]
    )
    ackermann_mux_node = Node(
        package='ackermann_mux',
        executable='ackermann_mux',
        name='ackermann_mux',
        parameters=[mux_param],
        remappings=[('/ackermann_cmd_out', '/ackermann_cmd_output')]
    )


    ld = LaunchDescription([])
    ld.add_action(ackermann_to_vesc_node)
    ld.add_action(vesc_to_odom_node)
    ld.add_action(vesc_driver_node)
    ld.add_action(ackermann_mux_node)
    
    return ld
