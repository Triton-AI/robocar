from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from base_common import get_share_file
import os
from environs import Env

env = Env()
env.read_env("race.env")

# launch_dir = get_package_share_directory("autonomy_launch")
# rvc_dir = get_package_share_directory("race_vehicle_controller")
# mcc_dir = get_package_share_directory("manual_control_converter")
# rde_dir = get_package_share_directory("race_decision_engine")
# rpp_dir = get_package_share_directory("race_path_planner")
# vks_dir = get_package_share_directory("vehicle_kinematic_state")
# bs_dir = get_package_share_directory("basestation_race_control")
# udp_dir = get_package_share_directory("udp_telemetry")
# telem_dir = get_package_share_directory("race_telemetry")
# bpp_dir = get_package_share_directory("boundary_publisher")
# jm_dir = get_package_share_directory("joystick_multiplexer")

# launch_mcc = DeclareLaunchArgument(
#     "launch_mcc",
#     default_value=str(env.bool("LAUNCH_MCC")),
#     description="If we should use MCC in place of RVC",
# )

# launch_vks = DeclareLaunchArgument(
#     "launch_vks", default_value=str(env.bool("LAUNCH_VKS")), description="If we should use VKS"
# )

# launch_udp = DeclareLaunchArgument(
#     "launch_udp",
#     default_value=str(env.bool("LAUNCH_UDP")),
#     description="If we should use UDP nodes",
# )

# launch_bpp = DeclareLaunchArgument(
#     "launch_bpp",
#     default_value=str(env.bool("LAUNCH_BPP")),
#     description="If we should use BPP",
# )


# rde_launch = IncludeLaunchDescription(
#     PythonLaunchDescriptionSource(
#         os.path.join(rde_dir, "launch", "race_decision_engine.launch.py")
#     ),
#     launch_arguments={
#         "ttl_dir": TextSubstitution(
#             text=get_share_file("race_metadata", "ttls", env.str("TTL_FOLDER"))
#         ),
#         "race_type": TextSubstitution(text=env.str("RACE_TYPE")),
#         "use_sim_time": TextSubstitution(text=str(env.bool("USE_SIM_TIME"))),
#     }.items(),
# )

# rpp_launch = IncludeLaunchDescription(
#     PythonLaunchDescriptionSource(os.path.join(rpp_dir, "launch", "race_path_planner.launch.py")),
#     launch_arguments={
#         "ttl_dir": TextSubstitution(
#             text=get_share_file("race_metadata", "ttls", env.str("TTL_FOLDER"))
#         ),
#         "race_type": TextSubstitution(text=env.str("RACE_TYPE")),
#         "use_sim_time": TextSubstitution(text=str(env.bool("USE_SIM_TIME"))),
#     }.items(),
# )

# bpp_launch = IncludeLaunchDescription(
#     PythonLaunchDescriptionSource(os.path.join(bpp_dir, "launch", "boundary_publisher.launch.py")),
#     condition=IfCondition(LaunchConfiguration("launch_bpp")),
#     launch_arguments={
#         "ttl_dir": TextSubstitution(
#             text=get_share_file("race_metadata", "ttls", env.str("TTL_FOLDER"))
#         ),
#         "race_type": TextSubstitution(text=env.str("RACE_TYPE")),
#         "use_sim_time": TextSubstitution(text=str(env.bool("USE_SIM_TIME"))),
#     }.items(),
# )

# rvc_launch = IncludeLaunchDescription(
#     PythonLaunchDescriptionSource(os.path.join(rvc_dir, "launch", "rvc.launch.py")),
#     condition=UnlessCondition(LaunchConfiguration("launch_mcc")),
#     launch_arguments={
#         "ttl_dir": TextSubstitution(
#             text=get_share_file("race_metadata", "ttls", env.str("TTL_FOLDER"))
#         ),
#         "vehicle_name": TextSubstitution(text=env.str("VEHICLE_NAME")),
#         "use_sim_time": TextSubstitution(text=str(env.bool("USE_SIM_TIME"))),
#     }.items(),
# )

# mcc_launch = IncludeLaunchDescription(
#     PythonLaunchDescriptionSource(
#         os.path.join(mcc_dir, "launch", "manual_control_converter.launch.py")
#     ),
#     condition=IfCondition(LaunchConfiguration("launch_mcc")),
#     launch_arguments={
#         "use_sim_time": TextSubstitution(text=str(env.bool("USE_SIM_TIME"))),
#     }.items(),
# )

# vks_launch = IncludeLaunchDescription(
#     PythonLaunchDescriptionSource(
#         os.path.join(vks_dir, "launch", "vehicle_kinematic_state.launch.py")
#     ),
#     condition=IfCondition(LaunchConfiguration("launch_vks")),
#     launch_arguments={
#         "use_sim_time": TextSubstitution(text=str(env.bool("USE_SIM_TIME"))),
#         "vehicle_name": TextSubstitution(text=env.str("VEHICLE_NAME")),
#         "origin_lat": TextSubstitution(text=str(env.float("GPS_ORIGIN_LAT"))),
#         "origin_lon": TextSubstitution(text=str(env.float("GPS_ORIGIN_LON"))),
#         "origin_alt": TextSubstitution(text=str(env.float("GPS_ORIGIN_ALT"))),
#         "use_kalman": TextSubstitution(text=str(env.bool("LAUNCH_VKS_EKF"))),
#     }.items(),
# )

# telem_launch = IncludeLaunchDescription(
#     PythonLaunchDescriptionSource(os.path.join(telem_dir, "launch", "telemetry.launch.py")),
#     launch_arguments={
#         "ttl_dir": TextSubstitution(
#             text=get_share_file("race_metadata", "ttls", env.str("TTL_FOLDER"))
#         ),
#         "vehicle_name": TextSubstitution(text=env.str("VEHICLE_NAME")),
#         "use_sim_time": TextSubstitution(text=str(env.bool("USE_SIM_TIME"))),
#     }.items(),
# )

# udp_launch = IncludeLaunchDescription(
#     PythonLaunchDescriptionSource(os.path.join(udp_dir, "launch", "udp_telemetry.launch.py")),
#     condition=IfCondition(LaunchConfiguration("launch_udp")),
#     launch_arguments={
#         "vehicle_name": TextSubstitution(text=env.str("VEHICLE_NAME")),
#         "use_sim_time": TextSubstitution(text=str(env.bool("USE_SIM_TIME"))),
#     }.items(),
# )

# jm_launch = IncludeLaunchDescription(
#     PythonLaunchDescriptionSource(
#         os.path.join(jm_dir, "launch", "joystick_multiplexer.launch.py")
#     ),
#     launch_arguments={
#         "vehicle_name": TextSubstitution(text=env.str("VEHICLE_NAME")),
#         "use_sim_time": TextSubstitution(text=str(env.bool("USE_SIM_TIME"))),
#     }.items(),
# )


def generate_launch_description():
    return LaunchDescription(
        [
            # launch_mcc,
            # launch_vks,
            # launch_udp,
            # launch_bpp,
            # vks_launch,
            # rde_launch,
            # rpp_launch,
            # rvc_launch,
            # mcc_launch,
            # udp_launch,
            # telem_launch,
            # bpp_launch,
            # jm_launch,
        ]
    )
