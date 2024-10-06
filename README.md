# robocar

## ALWAYS SOURCE ROS IN EVERY TERMINAL

## To get started
### First build
Clone this repo.

Then, in the robocar home directory, do
```
source docker.sh
```
This creates a new container based on `ghcr.io/triton-ai/robocar:foxy-x86`, and you should be in the docker container, if not, do `docker exec -it ucsd_ros bash` to get into it.

`car_config.yaml` contains the car's components. This includes lidar, oakd camera, etc.

- `node_config.yaml` contains selections of nodes and launch files to be used.
    - `SLAM` will be outputting a `pgm` or `png` file.
    - `TUM_global_planner` will be turned to `1` only after `SLAM` is used. It will be used once. The output will be global waypoints in `.json` format. 

- Then go to `map_select.yaml` to specify the new map to be used. 

TODO:
- vehicle dynamic


Next, while inside the container, do
```
cd /home/jetson/projects/robocar
sudo apt update && sudo apt upgrade
source source_ros2.sh
```
`source source_ros2.sh` helps you source all required environment while making sure some dependencies are indeed installed.
Don't worry if there's any error regarding no directory at: `install/setup.bash`, since you have not build the required packages yet.
Some useful custom commands can be used after sourcing `source_ros2.sh`:
- `source_ros2` sets up/sources required ROS environments.
- `build_ros2` colcon build all packages (except `livox_sdk2`, `livox_ros_driver2`), and then, source the environments.
- `build_ros2_pkg $arg1 $arg2 ...` colcon build the specific packages (replace `$arg#` with desired package name), and then, source the environments.

Import, and Build every packages (except Livox ones)
```bash
source_ros2 
make racer

vcs import < repos/ADDITIONAL_REPO_FILE (If you have one, please put it into "repos/")

make rosdep-install
build_ros2
```

#### If using Livox LiDARs,
Import, and Build `livox_sdk2` and `livox_ros_driver2`
```bash
source_ros2
make livox-driver
```
#### If using SICK LiDARs,
Import, and Build
```bash
source_ros2
make sick-driver
```

## Base Structure
1. **Master Stack**: `src/launch/basestation_launch/`
    - `nano src/launch/basestation_launch/param/car_config.yaml` to turn on/off any component of the racer
    - `nano src/launch/basestation_launch/param/node_config.yaml` to turn on/off any nodes you want

2. **Libraries**: `src/external/`
    - any external repo you clone will be here.

3. **Control - _Low level_**: `src/actuator/`

4. **Planning** & **Control - _High level_**: `src/planner/`