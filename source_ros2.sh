#!/bin/bash

# Display help
if [[ $1 == "-h" ]] || [[ $1 == "--help" ]]
then
  echo "Usage: source_ros2"
  echo
  echo "Sources the ROS distribution and workspace at WORKSPACE."
  echo 
  echo "containing the name of the target ROS distribution."
  echo "Supported distros: foxy"
  return
fi

export ROS_DISTRO=foxy
# sudo apt update && sudo apt upgrade
sudo apt install ros-$ROS_DISTRO-rmw-cyclonedds-cpp -y
sudo apt install ros-$ROS_DISTRO-rosbridge-suite -y
# sudo apt install ros-$ROS_DISTRO-nav2* -y
# sudo apt install ros-$ROS_DISTRO-spatio-temporal-voxel-layer -y

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

cd ${WORKSPACE}
source /opt/ros/$ROS_DISTRO/setup.bash
source install/setup.bash
source src/external/sensors/lidar/install/setup.bash

function source_ros2()
{
	cd ${WORKSPACE}
	source /opt/ros/$ROS_DISTRO/setup.bash
	source install/setup.bash
	source src/external/sensors/lidar/install/setup.bash
}

function build_ros2()
{
	cd ${WORKSPACE}
	colcon build --packages-ignore livox_ros_driver2 livox_sdk2 f1tenth_nav2_ctrl_plugin foxy_addon_behavior_tree --cmake-args -DCMAKE_BUILD_TYPE=Debug
	source install/setup.bash
}

function build_ros2_pkg()
{
	cd ${WORKSPACE}
	for arg in "$@"
	do	
		if [[ $arg == "livox_ros_driver2" ]] || [[ $arg == "livox_sdk2" ]]
		then
		echo "Don't try to colcon build livox packages"
		return
		fi
		if [[ $arg == "gb_opt" ]]
		then
		make gb_opt
		return
		fi
		if [[ $arg == "sick_scan_xd" ]]
		then
		make sick-driver
		return
		fi
	done
	colcon build --packages-select $@ --cmake-args -DCMAKE_BUILD_TYPE=Debug
	source install/setup.bash
}
complete -W "basestation_launch vesc_odom sensors planner global_planner state_estimation" build_ros2_pkg

function rmw_switch()
{
	if [[ $1 == "cyclonedds" ]]
	then
		export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
	else
		export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
	fi
}
complete -W "cyclonedds fastrtps" rmw_switch
