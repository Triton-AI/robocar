from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetLaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from base_common import get_share_file
import os
from environs import Env

env = Env()
env.read_env("race.env")

launch_dir = get_package_share_directory("robocar_launch")

launch_gps = DeclareLaunchArgument(
    "launch_gps",
    default_value="False",
    description="If we should use gps",
)

launch_lidar = DeclareLaunchArgument(
    "launch_lidar",
    default_value="False",
    description="If we should use lidar",
)

launch_camera = DeclareLaunchArgument(
    "launch_camera",
    default_value="False",
    description="If we should use camera",
)

SENSORS = env.str("SENSORS", "GPS_LIDAR")
if "GPS" in SENSORS:
    launch_gps = SetLaunchConfiguration("launch_gps", "True")

if "LIDAR" in SENSORS:
    launch_lidar = SetLaunchConfiguration("launch_lidar", "True")

if "CAMERA" in SENSORS:
    launch_camera = SetLaunchConfiguration("launch_camera", "True")


p1_gnss_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(launch_dir, "launch", "p1_gnss.launch.py")
    ),
    condition=IfCondition(LaunchConfiguration("launch_gps")),
    launch_arguments={
        "vehicle_name": TextSubstitution(text=env.str("VEHICLE_NAME")),
    }.items(),
)


def generate_launch_description():
    return LaunchDescription(
        [
            launch_gps,
            launch_lidar,
            launch_camera,
            p1_gnss_launch,
        ]
    )
