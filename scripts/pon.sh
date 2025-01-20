#!/bin/bash

# All documentations related are as follows
# https://github.com/PointOneNav/p1-host-tools
# https://github.com/PointOneNav/ros2-fusion-engine-driver/tree/main?tab=readme-ov-file#requirements
# Property of Triton AI @ UCSD, Kevin Shin

# Check for C++11 or later
if g++ -std=c++11 -E -x c++ - </dev/null &> /dev/null; then
	echo "C++11 or later: Installed"
else
	echo "C++11 or later: Not installed"
fi

# Check for CMake 3.x
if command -v cmake >/dev/null 2>&1; then
	cmake_version=$(cmake --version | head -n1 | awk '{print $3}')
	if [[ $cmake_version == 3.* ]]; then
		echo "CMake 3.x: Installed (version $cmake_version)"
       	else
	       	echo "CMake 3.x: Not installed (found version $cmake_version)"
	fi
else
	echo "CMake 3.x: Not installed"
fi

# Check for GCC or Clang
if command -v gcc >/dev/null 2>&1; then
	echo "GCC: Installed ($(gcc --version | head -n1))"
elif command -v clang >/dev/null 2>&1; then
	echo "Clang: Installed ($(clang --version | head -n1))"
else
	echo "Neither GCC nor Clang is installed"
fi

# Check for ROS 2 Foxy
if [ -f /opt/ros/foxy/setup.bash ]; then
	echo "ROS 2 Foxy: Installed"
else
	echo "ROS 2 Foxy: Not installed"
fi


# Install missing packages
install_package() {
       	echo "Installing $1..."
	sudo apt-get update
	sudo apt-get install -y $1
}

check_package() {
	if dpkg -s $1 >/dev/null 2>&1; then
		return 0
	else
		return 1
	fi
}

packages=("ros-foxy-gps-msgs" "ros-foxy-nmea-msgs" "ros-foxy-mavros-msgs" "ros-foxy-rtcm-msgs")
all_installed=true

for package in "${packages[@]}"; do
	if ! dpkg -s $package >/dev/null 2>&1; then
		install_package $package
       	else
		echo "$package is already installed."
	fi

	if ! check_package $package; then
		all_installed=false
	fi
done

# Configurations
if $all_installed; then
	echo "All required ROS2 Foxy packages are installed. Proceeding with FusionEngine and NTRIPClient installation."
	cd src/external/sensors/gps/pointonenav/
	cd p1-host-tools
	python3 -m venv p1_tools_venv
	source p1_tools/bin/activate
	pip3 install -r requirements.txt	
	python3 bin/config_tool.py apply uart2_message_rate fe ROSPoseMessage 100ms
	python3 bin/config_tool.py apply uart2_message_rate fe ROSGPSFixMessage 100ms
	python3 bin/config_tool.py apply uart2_message_rate fe ROSIMUMessage 100ms
	python3 bin/config_tool.py save
	deactivate
	
	cd ../fusion-engine-driver
	rosdep install -i --from-path ./ --rosdistro foxy -y
	colcon build --packages-select fusion-engine-driver
	source install/local_setup.bash
	
	cd ../ntrip_client
	colcon build --packages-select ntrip_client
	source install/local_setup.bash

	echo "FusionEngine and NTRIPClient configured and available as ROS2 packages."
else
	echo "Some required ROS2 Foxy packages are missing. Please install them before proceeding"
fi
