#!/bin/bash

cd ${ROBOCAR_DIR} && source ${ROBOCAR_DIR}/src/tools/startup_services/launch/base_source.bash
ros2 launch basestation_launch autonomy_all.launch.py