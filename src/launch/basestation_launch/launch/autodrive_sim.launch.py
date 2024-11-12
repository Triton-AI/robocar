from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def generate_launch_description():
    pkg_name = 'basestation_launch'
    desc_dir = get_package_share_directory(pkg_name)

    config = os.path.join(
        desc_dir,
        'param',
        'autodrive_sim.yaml'
    )
    
    config_dict = yaml.safe_load(open(config, 'r'))
    # has_opp = config_dict['bridge']['ros__parameters']['num_agent'] > 1
    # teleop = config_dict['bridge']['ros__parameters']['kb_teleop']

    bridge_node = Node(
        package='autodrive_f1tenth',
        executable='autodrive_bridge',
        name='autodrive_bridge',
        emulate_tty=True,
        output='screen',
        remappings=[
            ('/autodrive/f1tenth_1/lidar', '/ego_racecar/scan'),
            ('/autodrive/f1tenth_1/front_camera', '/ego_racecar/front_camera'),
            ('/autodrive/f1tenth_1/imu', '/ego_racecar/imu'),
        ],
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', [FindPackageShare("autodrive_f1tenth"), '/rviz', '/simulator.rviz',]]
        # arguments=['-d', os.path.join(get_package_share_directory('autodrive_f1tenth'), '/rviz', 'simulator.rviz')]
    )
    # map_server_node = Node(
    #     package='nav2_map_server',
    #     executable='map_server',
    #     parameters=[{'yaml_filename': config_dict['bridge']['ros__parameters']['map_path'] + '.yaml'},
    #                 {'topic': 'map'},
    #                 {'frame_id': 'map'},
    #                 {'output': 'screen'},
    #                 {'use_sim_time': True}]
    # )
    # nav_lifecycle_node = Node(
    #     package='nav2_lifecycle_manager',
    #     executable='lifecycle_manager',
    #     name='lifecycle_manager_localization',
    #     output='screen',
    #     parameters=[{'use_sim_time': True},
    #                 {'autostart': True},
    #                 {'node_names': ['map_server']}]
    # )
    ackermann_odom_node = Node(
        package='ackermann_odom',
        executable='ackermann_odom_node',
        name='ackermann_odom',
        # prefix=['gdb -ex run --args'],  # For GDB within the launch terminal
        parameters=[{'use_sim_time': False}],
    )
    ego_robot_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='ego_robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', os.path.join(get_package_share_directory('basestation_launch'), 'urdf', 'autodrive_racecar.xacro')])}],
        remappings=[
            ('/robot_description', '/ego_racecar/robot_description'),
            ('/joint_states', '/ego_racecar/joint_states'),
        ]
    )
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='ego_joint_state_publisher',
        arguments=[os.path.join(get_package_share_directory('basestation_launch'), 'urdf', 'autodrive_racecar.xacro')],
        # condition=UnlessCondition(LaunchConfiguration('gui'))
        remappings=[
            ('/robot_description', '/ego_racecar/robot_description'),
            ('/joint_states', '/ego_racecar/joint_states'),
        ],
    )

    ld = LaunchDescription()

    # finalize
    ld.add_action(bridge_node)
    # ld.add_action(nav_lifecycle_node)
    # ld.add_action(map_server_node)
    ld.add_action(ackermann_odom_node)

    if config_dict['bridge']['ros__parameters']['launch_rviz']:
        ld.add_action(rviz_node)

    if config_dict['bridge']['ros__parameters']['ego_urdf_pub']:
        ld.add_action(ego_robot_publisher)
        ld.add_action(joint_state_publisher_node)
    # if has_opp:
    #     if config_dict['bridge']['ros__parameters']['opp_urdf_pub']:
    #         ld.add_action(opp_robot_publisher)

    return ld