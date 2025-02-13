# Copyright 2025 AI Racing Tech
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
ARG BASE_IMAGE

FROM $BASE_IMAGE as base
LABEL maintainer="Winston Chou <winston.h.chou@gmail.com>"
SHELL ["/bin/bash", "-o", "pipefail", "-ic"]

ARG APT_FILE
ARG APT_GPU_FILE
ARG PIP_FILE
ARG PIP_GPU_FILE
ARG PYTORCH_FILE
ARG CUSTOM_INSTALL_FILE

## Install apt packages
RUN apt update && DEBIAN_FRONTEND=noninteractive apt -y install --no-install-recommends \
  ca-certificates \
  && apt clean \
  && rm -rf /var/lib/apt/lists/*

RUN apt update && DEBIAN_FRONTEND=noninteractive apt -y install --no-install-recommends \
  software-properties-common \
  && apt clean \
  && rm -rf /var/lib/apt/lists/*

COPY tools/image/${APT_FILE} /robocar_tmp/${APT_FILE}

RUN apt update && DEBIAN_FRONTEND=noninteractive apt -y install --no-install-recommends \
        $(cat /robocar_tmp/${APT_FILE} | cut -d# -f1) && \
    apt clean && \
    rm -rf /var/lib/apt/lists/* /robocar_tmp/${APT_FILE}

COPY tools/image/${APT_GPU_FILE} /robocar_tmp/${APT_GPU_FILE}
RUN apt update && DEBIAN_FRONTEND=noninteractive apt -y install --no-install-recommends \
        $(cat /robocar_tmp/${APT_GPU_FILE} | cut -d# -f1) && \
    apt clean && \
    rm -rf /var/lib/apt/lists/* /robocar_tmp/${APT_GPU_FILE}

## Add GitHub to known hosts for private repositories
RUN mkdir -p ~/.ssh \
  && ssh-keyscan github.com >> ~/.ssh/known_hosts \
  && ssh-keyscan gitlab.com >> ~/.ssh/known_hosts

RUN python3 -m pip install --upgrade pip

COPY tools/image/${PIP_FILE} /robocar_tmp/${PIP_FILE}
RUN DEBIAN_FRONTEND=noninteractive pip3 install -U --no-cache --ignore-installed -r /robocar_tmp/${PIP_FILE} && \
    rm -rf /robocar_tmp/${PIP_FILE}

COPY tools/image/${PIP_GPU_FILE} /robocar_tmp/${PIP_GPU_FILE}
RUN DEBIAN_FRONTEND=noninteractive pip3 install -U --no-cache --ignore-installed -r /robocar_tmp/${PIP_GPU_FILE} && \
    rm -rf /robocar_tmp/${PIP_GPU_FILE}

COPY tools/image/${PYTORCH_FILE} /robocar_tmp/${PYTORCH_FILE}
RUN DEBIAN_FRONTEND=noninteractive pip3 install -U --no-cache --ignore-installed -r /robocar_tmp/${PYTORCH_FILE} && \
    rm -rf /robocar_tmp/${PYTORCH_FILE}

RUN rm -rf /robocar_tmp

FROM base as base_custom

ARG ROS_DISTRO
ENV ROS_DISTRO=${ROS_DISTRO}

COPY tools/image/${CUSTOM_INSTALL_FILE} /robocar_tmp/${CUSTOM_INSTALL_FILE}
RUN --mount=type=ssh \
    DEBIAN_FRONTEND=noninteractive /bin/sh /robocar_tmp/${CUSTOM_INSTALL_FILE} && \
    rm -rf /robocar_tmp/${CUSTOM_INSTALL_FILE}

FROM base_custom as base_ros

ARG ROS_DISTRO
ARG ROS_INSTALL

# Install Ros Distro
COPY tools/image/${ROS_INSTALL} /robocar_tmp/${ROS_INSTALL}
RUN DEBIAN_FRONTEND=noninteractive /bin/bash /robocar_tmp/${ROS_INSTALL} ${ROS_DISTRO} && \
    rm -rf /robocar_tmp/${ROS_INSTALL}

FROM base_ros as robocar_deps_generator

ARG ROS_DISTRO
ARG ROS_SOURCE
ARG VCS_IMPORTS_SCRIPT
ARG SKIP_KEYS

COPY src/ /workspace/src
COPY *.repos /workspace
COPY tools/image/${VCS_IMPORTS_SCRIPT} /workspace/${VCS_IMPORTS_SCRIPT}

RUN --mount=type=ssh \
    source /opt/ros/${ROS_SOURCE}/setup.bash && \
    apt update && \
    rosdep update --include-eol-distros && \
    cd /workspace && \
    rm -rf src/external && \
    DEBIAN_FRONTEND=noninteractive ./${VCS_IMPORTS_SCRIPT} && \
    DEBIAN_FRONTEND=noninteractive rosdep install --as-root "apt:false pip:false" --simulate --reinstall -r -y --ignore-src --rosdistro ${ROS_DISTRO} --skip-keys ${SKIP_KEYS} --from-paths . | sort >> /ros-deps && \
    cd / && \
    rm -rf /workspace

FROM base_ros as robocar_deps

ARG ROS_DISTRO

COPY --from=robocar_deps_generator /ros-deps .

RUN cat /ros-deps

RUN apt update && \
    rosdep update --include-eol-distros && \
    DEBIAN_FRONTEND=noninteractive /bin/bash ros-deps && \
    apt clean && \
    rm -rf /var/lib/apt/lists/* /ros-deps

FROM robocar_deps as robocar_image

ARG ROS_DISTRO
ARG ROS_SOURCE
ARG EXPORTS_SCRIPT
ARG EXPORTS_GPU_SCRIPT

ENV ROS_DISTRO=${ROS_DISTRO}

COPY tools/image/${EXPORTS_SCRIPT} /robocar_tmp/${EXPORTS_SCRIPT}
COPY tools/image/${EXPORTS_GPU_SCRIPT} /robocar_tmp/${EXPORTS_GPU_SCRIPT}
RUN  cd /robocar_tmp && \
    ./${EXPORTS_GPU_SCRIPT} && \
    ./${EXPORTS_SCRIPT} && \
    rm -rf /robocar_tmp/

COPY tools/image/cyclonedds.xml /etc/cyclone/cyclonedds.xml

# FROM robocar_image as robocar_image_built

# ARG VCS_IMPORTS_SCRIPT
# ARG RC_BUILD_TARGET
# ARG ROS_SOURCE

# COPY src/ /opt/robocar/src
# COPY *.repos /opt/robocar
# COPY tools/ /opt/robocar/tools
# COPY Makefile /opt/robocar/Makefile

# RUN --mount=type=ssh \
#     --mount=type=cache,target=/opt/robocar/build \
#     source /opt/ros/${ROS_SOURCE}/setup.bash && \
#     cd /opt/robocar && \
#     rm -rf src/external && \
#     DEBIAN_FRONTEND=noninteractive ./tools/image/${VCS_IMPORTS_SCRIPT} && \
#     export PATH="/opt/cmake-install/bin:${PATH}" && \
#     export CMAKE_PREFIX_PATH="/opt/cmake-install:${CMAKE_PREFIX_PATH}" && \
#     export LD_LIBRARY_PATH="${LD_LIBRARY_PATH}:/usr/local/lib" && \
#     DEBIAN_FRONTEND=noninteractive make ${RC_BUILD_TARGET} && \
#     rm -rf src

# RUN echo "source /opt/robocar/install/setup.bash" >> /etc/bash.bashrc

CMD ["/bin/bash", "-l", "-c", "exec /bin/bash"]

