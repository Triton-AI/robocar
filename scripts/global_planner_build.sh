#!/bin/bash

cd src/path_planning/global_planner/
python3 -m venv venv
source venv/bin/activate
pip3 install -r global_planner/global_racetrajectory_optimization/requirements.txt
cd ../../..

source /opt/ros/foxy/setup.bash
colcon build --packages-select global_planner --cmake-args -DCMAKE_BUILD_TYPE=Debug
deactivate

cd src/path_planning/global_planner/
rm -rf venv
cd ../../..

source install/setup.bash