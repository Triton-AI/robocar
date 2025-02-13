#!/bin/bash

# Display help
if [[ $1 == "-h" ]] || [[ $1 == "--help" ]]
then
  echo "Usage: source tools/scripts/source_all.sh"
  echo
  echo "Sources the ROS distribution and workspace at WORKSPACE."
  echo 
  echo "containing the name of the target ROS distribution."
  return
fi

#!/bin/bash

ROS_DISTRO_SOURCE=/opt/ros/${ROS_DISTRO}/setup.bash
if [ -f "${ROS_DISTRO_SOURCE}" ]; then
    source ${ROS_DISTRO_SOURCE}
fi

FRC_ROS_LOCAL=install/setup.bash;
if [ -f "${FRC_ROS_LOCAL}" ]; then
    source ${FRC_ROS_LOCAL};
fi;

function RMW()
{
	if [[ $1 == "cyclonedds" ]]
	then
		export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
	elif [[ $1 == "fastdds" ]]
	then
		export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
	else
		echo "Current ROS Middleware: $RMW_IMPLEMENTATION"
	fi
}
complete -W "cyclonedds fastdds" RMW
