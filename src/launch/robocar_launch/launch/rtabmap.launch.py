import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

launch_dir = get_package_share_directory("robocar_launch")

def launch_setup(context, *args, **kwargs):
    name = LaunchConfiguration('name').perform(context)
    depthai_prefix = get_package_share_directory("depthai_ros_driver")

    params_file= LaunchConfiguration("params_file")
    parameters = [
        {
            "frame_id": name,
            "subscribe_depth": True,
            "approx_sync": True,
            "Rtabmap/DetectionRate": "3.5",
            "rgbd_cameras": 1,
        }
    ]

    remappings = [
        ("rgb/image", name+"/rgb/image_raw"),
        ("rgb/camera_info", name+"/rgb/camera_info"),
        ("depth/image", name+"/stereo/image_raw"),
    ]
    return [
        Node(
                package="rtabmap_odom",
                executable="rgbd_odometry",
                output="screen",
                parameters=parameters,
                remappings=remappings,
            ),
            Node(
                package="rtabmap_slam",
                executable="rtabmap",
                output="screen",
                parameters=parameters,
                remappings=remappings,
                arguments=["-d"],
            ),
            Node(
                package="rtabmap_viz",
                executable="rtabmap_viz",
                output="screen",
                parameters=parameters,
                remappings=remappings,
            ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, 'launch', 'oakd_camera_base.launch.py')),
            launch_arguments={"name": name,
                              "params_file": params_file}.items())
    ]


def generate_launch_description():
    depthai_prefix = get_package_share_directory("depthai_ros_driver")
    declared_arguments = [
        DeclareLaunchArgument("name", default_value="oak"),
        DeclareLaunchArgument("params_file", default_value=os.path.join(launch_dir, 'param/ucsd_blue', 'oak_d_pro_w.yaml')),
    ]

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
