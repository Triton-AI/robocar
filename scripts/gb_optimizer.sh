#!/bin/bash

source src/path_planning/gb_optimizer_nav2/gb_opt_env/bin/activate
pip3 install -r src/path_planning/gb_optimizer_nav2/gb_optimizer_nav2/global_racetrajectory_optimization/requirements.txt
source /opt/ros/foxy/setup.bash
colcon build --packages-select gb_optimizer_nav2 --cmake-args -DCMAKE_BUILD_TYPE=Debug
deactivate
source install/setup.bash