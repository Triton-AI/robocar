#!/bin/bash

# check if ROS_DISTRO is set
if [ -z ${ROS_DISTRO+x} ]; then
    echo "ROS_DISTRO variable is not set. Please set it to the ROS distro you are using (e.g. humble)"
    exit 1
fi

# add some env variables to /etc/default/frc-ros
echo "ROBOCAR_DIR=${PWD}" | sudo tee /etc/default/frc-ros
echo "ROS_DISTRO=${ROS_DISTRO}" | sudo tee -a /etc/default/frc-ros
echo "ROSBAG_DIR=${ROSBAG_DIR}" | sudo tee -a /etc/default/frc-ros
echo "CONT_NAME=frc-ros-container" | sudo tee -a /etc/default/frc-ros


# check if user is root
if [ "$EUID" -eq 0 ]; then
    echo "installing services for root user"

    # create symbolic links for all the services
    for file in ${PWD}/src/tools/startup_services/services/*.service; do
        sudo ln -s -f $file /etc/systemd/system/$(basename $file)
    done

    # update the systemd daemon
    systemctl daemon-reload

else
    echo "installing services for user"

    # create a folder for the user if it doesn't exist already
    if [ ! -d "~/.config/systemd/user/" ]; then
        sudo mkdir -p ~/.config/systemd/user
    fi

    # create symbolic links for all the services
    for file in ${PWD}/src/tools/startup_services/services/*.service; do
        sudo ln -s -f $file ~/.config/systemd/user/$(basename $file)
    done

    # create a folder for the user if it doesn't exist already
    if [ ! -d "~/.config/systemd/user/default.target.wants" ]; then
        sudo mkdir -p ~/.config/systemd/user/default.target.wants
    fi

    # create a symlink in its WantedBy directory
    for file in ${PWD}/src/tools/startup_services/services/*.service; do
        sudo ln -s -f $file ~/.config/systemd/user/default.target.wants/$(basename $file)
    done

    # update the systemd daemon
    systemctl --user daemon-reload

fi