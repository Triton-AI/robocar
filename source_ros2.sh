#!/bin/bash

# Display help
if [[ $1 == "-h" ]] || [[ $1 == "--help" ]]
then
  echo "Usage: source_ros2"
  echo
  echo "Sources the ROS distribution and workspace at WORKSPACE_DIR."
  echo 
  echo "containing the name of the target ROS distribution."
  echo "Supported distros: foxy"
  return
fi

# sudo apt update && sudo apt upgrade
sudo apt install ros-foxy-rmw-cyclonedds-cpp -y
sudo apt install ros-foxy-rosbridge-suite -y
# sudo apt install ros-foxy-nav2* -y
# sudo apt install ros-foxy-spatio-temporal-voxel-layer -y

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

cd /home/jetson/projects/robocar
source /opt/ros/foxy/setup.bash
source install/setup.bash
source src/external/lidar/install/setup.bash

function source_ros2()
{
	cd /home/jetson/projects/robocar
	source /opt/ros/foxy/setup.bash
	source install/setup.bash
	source src/external/lidar/install/setup.bash
}

function build_ros2()
{
	cd /home/jetson/projects/robocar
	colcon build --packages-ignore livox_ros_driver2 livox_sdk2 f1tenth_nav2_ctrl_plugin --cmake-args -DCMAKE_BUILD_TYPE=Debug
	source install/setup.bash
}

function build_ros2_pkg()
{
	cd /home/jetson/projects/robocar
	for arg in "$@"
	do	
		if [[ $arg == "livox_ros_driver2" ]] || [[ $arg == "livox_sdk2" ]]
		then
		echo "Don't try to colcon build livox packages"
		return
		fi
		if [[ $arg == "gb_optimizer_nav2" ]]
		then
		make gb_optimizer
		return
		fi
	done
	colcon build --packages-select $@ --cmake-args -DCMAKE_BUILD_TYPE=Debug
	source install/setup.bash
}
complete -W "basestation_launch vesc_odom sensors planner state_estimation" build_ros2_pkg

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
