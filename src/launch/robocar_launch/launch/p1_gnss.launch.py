import launch
import launch_ros.actions
from launch_ros.actions import Node
import launch.substitutions
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from base_common import check_val_in_list, get_share_file, to_lower
from launch.conditions import IfCondition


gps_remappings = {
    "fix": "/p1/fix",
    "gps_fix": "/p1/gps",
    "imu": "/p1/imu",
    "pose": "/p1/pose",
}.items()


def generate_launch_description():

    launch_dir = "robocar_launch"

    vehicle_name_arg = DeclareLaunchArgument("vehicle_name", default_value="UCSD_BLUE", description="Vehicle profile to use")

    param_file_path = (
        get_share_file(launch_dir),
        "/param/",
        to_lower(LaunchConfiguration("vehicle_name")),
        "/p1_gnss.param.yaml",
    )

    ntrip_client_node = Node(
        name='ntrip_client',
        package='ntrip_client',
        executable='ntrip_ros.py',
        parameters=[
            param_file_path,
            {
                'ssl': 'False',
                'cert': 'None',
                'key':  'None',
                'ca_cert': 'None',
                'rtcm_frame_id': 'odom',
                # Optional parameters that will allow for longer or shorter NMEA messages. Standard max length for NMEA is 82
                'nmea_max_length': 128,
                'nmea_min_length': 3,
                # Use this parameter to change the type of RTCM message published by the node. Defaults to "mavros_msgs", but we also support "rtcm_msgs"
                'rtcm_message_package': 'rtcm_msgs',
                # Will affect how many times the node will attempt to reconnect before exiting, and how long it will wait in between attempts when a reconnect occurs
                'reconnect_attempt_max': 10,
                'reconnect_attempt_wait_seconds': 5,
                # How many seconds is acceptable in between receiving RTCM. If RTCM is not received for this duration, the node will attempt to reconnect
                'rtcm_timeout_seconds': 4
            }
        ],
        remappings=gps_remappings,
    )

    fusion_engine_node = Node(
        package="fusion-engine-driver",
        executable="fusion_engine_ros_driver",
        name="fusion_engine_node",
        output="screen",
        parameters=[param_file_path],
        remappings=gps_remappings,
    )


    return LaunchDescription(
        [
            vehicle_name_arg,
            DeclareLaunchArgument('debug', default_value='false'),
            SetEnvironmentVariable(name='NTRIP_CLIENT_DEBUG', value=LaunchConfiguration('debug')),
            ntrip_client_node,
            fusion_engine_node,
        ]
    )