import launch
import launch_ros.actions
from launch_ros.actions import Node
import launch.substitutions
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from base_common import check_val_in_list, get_param_file
from launch.conditions import IfCondition
from environs import Env

def generate_launch_description():

    env = Env()
    env.read_env("race.env")
    controller_name = env.str("CONTROLLER_NAME")
    vehicle_name_arg = DeclareLaunchArgument("vehicle_name")

    # Default joystick translator params
    config = get_param_file(
        "basestation_launch", controller_name, "vehicle_name"
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy',
        parameters=[
            {"deadzone": 0.01},
            {"autorepeat_rate": 20.0},
            {"coalesce_interval": 0.01},
        ],
        remappings=[
            ('/joy', '/joystick/joy'),
            ('/joy/set_feedback', '/joystick/joy/set_feedback'),
        ],
    )

    joy_teleop_node = Node(
        package='joy_teleop',
        executable='joy_teleop',
        name='joy_teleop',
        parameters=[
            config,
        ],
        remappings=[
            ('/joy', '/joystick/joy'),
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

    return launch.LaunchDescription(
        [
            vehicle_name_arg,
            joy_node,
            joy_teleop_node,
        ]
    )
