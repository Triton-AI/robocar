#!/bin/bash

pushd `pwd` > /dev/null

LIVOX_SDK=src/external/sensors/lidar/livox/livox-SDK2
if [ -d $LIVOX_SDK ]; then
	cd $LIVOX_SDK
	mkdir build
	cd build
	cmake .. && make -j3
	sudo make install
fi

popd > /dev/null

pushd `pwd` > /dev/null

LIVOX_ROS_DRIVER=src/external/sensors/lidar/livox/livox_ros_driver2
if [ -d $LIVOX_ROS_DRIVER ]; then
	cd $LIVOX_ROS_DRIVER
	./build.sh ROS2
fi

popd > /dev/null
