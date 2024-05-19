#! /bin/bash

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
echo $SCRIPT_DIR
cd $SCRIPT_DIR && pip3 install -e ../src/path_planning/global_planner/global_planner/global_racetrajectory_optimization/