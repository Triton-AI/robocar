from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from base_common import get_share_file
import os
from environs import Env

env = Env()
env.read_env("race.env")

launch_dir = get_package_share_directory("basestation_launch")
js_dir = get_package_share_directory("basestation_launch")


launch_joystick = DeclareLaunchArgument(
    "launch_joystick",
    default_value=str(env.bool("LAUNCH_JOYSTICK")),
    description="If we should use joystick",
)


js_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(js_dir, "launch", "joy.launch.py")
    ),
    condition=IfCondition(LaunchConfiguration("launch_joystick")),
    launch_arguments={
        "vehicle_name": TextSubstitution(text=env.str("VEHICLE_NAME")),
    }.items(),
)


# vis_launch = IncludeLaunchDescription(
#     PythonLaunchDescriptionSource(os.path.join(rt_dir, "launch", "visualization.launch.py")),
#     launch_arguments={
#         "vehicle_name": TextSubstitution(text=env.str("VEHICLE_NAME")),
#         "use_sim_time": TextSubstitution(text=str(env.bool("USE_SIM_TIME"))),
#     }.items(),
# )


def generate_launch_description():
    return LaunchDescription(
        [
            launch_joystick,
            js_launch,
            # vis_launch,
        ]
    )
