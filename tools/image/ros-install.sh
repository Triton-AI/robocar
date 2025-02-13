#!/bin/bash

sudo add-apt-repository universe
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

apt update && \ 
    apt install -y --no-install-recommends \
    ros-$1-desktop \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-vcstool \
    python3-rosdep && \
    apt clean && \
    rm -rf /var/lib/apt/lists/*

colcon mixin add default \
    https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml && \
    colcon mixin update && \
    colcon metadata add default \
    https://raw.githubusercontent.com/colcon/colcon-metadata-repository/master/index.yaml && \
    colcon metadata update

sudo rosdep init
