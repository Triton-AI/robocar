import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

launch_dir = get_package_share_directory("autonomy_launch")

autonomy_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(launch_dir, "launch", "autonomy_core.launch.py"))
)

misc_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(launch_dir, "launch", "misc.launch.py"))
)


def generate_launch_description():
    return LaunchDescription(
        [
            autonomy_launch,
            misc_launch,
        ]
    )
