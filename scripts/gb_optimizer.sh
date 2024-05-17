#!/bin/bash

cd src/path_planning/gb_optimizer_nav2/
python3 -m venv venv --system-site-packages --symlinks
source venv/bin/activate
pip3 install -r gb_optimizer_nav2/global_racetrajectory_optimization/requirements.txt
cd ../../..

source /opt/ros/foxy/setup.bash
colcon build --packages-select gb_optimizer_nav2 --cmake-args -DCMAKE_BUILD_TYPE=Debug
deactivate

cd src/path_planning/gb_optimizer_nav2/
rm -rf venv
cd ../../..

source install/setup.bash