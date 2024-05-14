#!/bin/bash

xhost +
docker run \
    --name sim_test \
    -it \
    --rm \
    --privileged \
    --net=host \
    -e DISPLAY=$DISPLAY \
    -v /dev/bus/usb:/dev/bus/usb \
    --device /dev/video0 \
    -v  /run/user/1000/gdm/Xauthority:/root/.Xauthority:rw \
    f1tenth_x86:latest
