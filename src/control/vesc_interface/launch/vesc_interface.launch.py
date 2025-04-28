from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition
from base_common import get_share_file, check_val_in_list, to_lower
from environs import Env

def generate_launch_description():
    launch_dir = 'vesc_interface'

    vehicle_name_arg = DeclareLaunchArgument("vehicle_name", default_value="UCSD_BLUE")

    vesc_param = (
        get_share_file(launch_dir),
        "/param/",
        to_lower(LaunchConfiguration("vehicle_name")),
        "/vesc_interface.param.yaml",
    )

    ackermann_vesc_param = get_share_file(launch_dir, 'param', 'ackermann_vesc.param.yaml')
    mux_param = get_share_file(launch_dir, 'param', 'ackermann_mux.param.yaml')

    ackermann_to_vesc_node = Node(
        package='vesc_ackermann',
        executable='ackermann_to_vesc_node',
        name='ackermann_to_vesc',
        parameters=[ackermann_vesc_param],
        remappings=[
            ('/ackermann_cmd', '/mux/ackermann_cmd'),
            ('/commands/motor/speed', '/vesc/commands/motor/speed'),
            ('/commands/servo/position', '/vesc/commands/servo/position'),
        ],
    )
    ackermann_mux_node = Node(
        package='ackermann_mux',
        executable='ackermann_mux',
        name='ackermann_mux',
        parameters=[mux_param],
        remappings=[
            ('/ackermann_cmd', '/mux/ackermann_cmd'),
        ],
    )


    vesc_to_odom_node = Node(
        package='vesc_ackermann',
        executable='vesc_to_odom_node',
        name='vesc_to_odom',
        parameters=[vesc_param],
        remappings=[
            ('/sensors/core', '/vesc/sensors/core'),
            ('/sensors/servo_position_command', '/vesc/sensors/servo_position_command'),
            ('/odom', '/vesc/odom'),
        ],
        condition=IfCondition(
            check_val_in_list(
                "vehicle_name",
                [
                    "UCSD_BLUE",
                    "UCSD_YELLOW",
                ],
            )
        ),
    )
    vesc_driver_node = Node(
        package='vesc_driver',
        executable='vesc_driver_node',
        name='vesc_driver',
        parameters=[vesc_param],
        remappings=[
            ('/sensors/core', '/vesc/sensors/core'),
            ('/sensors/imu', '/vesc/sensors/imu'),
            ('/sensors/imu/raw', '/vesc/sensors/imu/raw'),
            ('/sensors/servo_position_command', '/vesc/sensors/servo_position_command'),
            ('/commands/motor/brake', '/vesc/commands/motor/brake'),
            ('/commands/motor/current', '/vesc/commands/motor/current'),
            ('/commands/motor/duty_cycle', '/vesc/commands/motor/duty_cycle'),
            ('/commands/motor/position', '/vesc/commands/motor/position'),
            ('/commands/motor/speed', '/vesc/commands/motor/speed'),
            ('/commands/servo/position', '/vesc/commands/servo/position'),
        ],
        condition=IfCondition(
            check_val_in_list(
                "vehicle_name",
                [
                    "UCSD_BLUE",
                    "UCSD_YELLOW",
                ],
            )
        ),
    )


    ld = LaunchDescription([])
    ld.add_action(vehicle_name_arg)
    ld.add_action(ackermann_to_vesc_node)
    ld.add_action(vesc_to_odom_node)
    ld.add_action(vesc_driver_node)
    ld.add_action(ackermann_mux_node)
    
    return ld
