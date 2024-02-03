# robocar

## ALWAYS SOURCE ROS IN EVERY TERMINAL

## First build

```
vcs import < YOUR_REPOS_FILE
```

Then, in the robocar home directory.

```
source /opt/ros/foxy/setup.bash
vcs import < common.foxy.repos
vcs import < ADDITIONAL_REPO_FILE
make rosdep-install
make
source install/setup.bash
```

## Future build
In the home directory, `make`
