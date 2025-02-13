#!/bin/bash

cd ${ROBOCAR_DIR}
source race.env
if [ "${RUNTIME}" = "nvidia" ]; then
    echo "RUNTIME is set to nvidia"
else
    echo "RUNTIME is set to docker"
fi
make session IMG_NAME=ubuntu:20.04