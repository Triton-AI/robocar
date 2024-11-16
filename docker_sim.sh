#!/bin/bash

if [ -z "$XAUTH" ]
then
      echo "\$XAUTH is empty"
      return
else
      echo "\$XAUTH is NOT empty"
      xauth_path=$XAUTH
fi

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
    -v  $xauth_path:/root/.Xauthority:rw \
    ghcr.io/triton-ai/robocar:foxy-x86