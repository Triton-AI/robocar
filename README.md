# robocar

## ALWAYS SOURCE ROS IN EVERY TERMINAL

## First build
Clone this repo.

Then, in the robocar home directory, do
```bash
source tools/scripts/source_all.sh
make vcs-import VCS_FILE=robocar.foxy.repos
make vcs-import VCS_FILE=ADDITIONAL_REPO_FILE (If you have one)
make rosdep-install-eol
make
source tools/scripts/source_all.sh
```

## Future build
In the home directory, `make`
