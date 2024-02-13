# robocar

## ALWAYS SOURCE ROS IN EVERY TERMINAL

## First build
Clone this repo.

Then, in the robocar home directory, do
```
source /opt/ros/foxy/setup.bash
vcs import < common.repos
vcs import < ADDITIONAL_REPO_FILE (If you have one)
make rosdep-install
make
source install/setup.bash
```

## Future build
In the home directory, `make`
