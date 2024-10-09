# --------------------------------------------------------------------------------------------------------------------------------------------------
# Getting Base image (ubuntu 20.04)
# --------------------------------------------------------------------------------------------------------------------------------------------------
FROM ros:foxy
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

ARG WORKSPACE=/home/projects/f1tenth_ws/

WORKDIR /

####################################################################################################################################################
# Install required packages/dependencies for f1tenth_x86
####################################################################################################################################################
RUN apt-get update --fix-missing && \
    apt-get install -y --no-install-recommends sudo wget gedit curl unzip net-tools nano vim libeigen3-dev tmux ros-foxy-rviz2 && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-foxy-desktop \
    ros-foxy-message-filters \
    ros-foxy-image-transport \
    ros-foxy-teleop-twist-joy \
    ros-foxy-joy \
    ros-foxy-joy-teleop \
    ros-foxy-rviz-default-plugins \
    ros-foxy-rviz-rendering \
    ros-foxy-ros2bag \
    ros-foxy-rosbag2-converter-default-plugins \
    ros-foxy-rosbag2-storage-default-plugins \
    ros-foxy-robot-localization \
    ros-foxy-slam-toolbox \
    ros-foxy-ackermann-msgs \
    ros-foxy-serial-driver \
    ros-foxy-depthai-ros \
    ros-foxy-pcl-ros \
    ros-foxy-robot-state-publisher \
    ros-foxy-joint-state-publisher && \
    rm -rf /var/lib/apt/lists/*

# --------------------------------------------------------------------------------------------------------------------------------------------------
# Install useful packages
# --------------------------------------------------------------------------------------------------------------------------------------------------
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    python3-pip \
    python3-dev \
    python3-argcomplete \
    cmake \
    nano \
    iputils-ping \
    x11-apps \
    xauth \
    nautilus \
    usbutils \
    vim \
    tmux \
    tmuxp \
    network-manager \
    firefox \
    git-all \
    cheese \
    jstest-gtk \
    joystick \
    python3-tk \
    python3-rosdep \
    python3-vcstool \
    ros-dev-tools && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    ros-foxy-rmw-cyclonedds-cpp \
    ros-foxy-rosbridge-suite \
    ros-foxy-nav2* \
    ros-foxy-navigation2 \
    ros-foxy-smac-planner \
    ros-foxy-spatio-temporal-voxel-layer \
    ros-foxy-nonpersistent-voxel-layer \
    ros-foxy-tf-transformations \
    ros-foxy-imu-tools && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update --fix-missing && \
    apt-get install -y xvfb ffmpeg libgdal-dev libsm6 libxext6 && \
    apt-get -y dist-upgrade && \
    rm -rf /var/lib/apt/lists/*

# Install Python dependencies
RUN pip3 install attrdict numpy pillow opencv-contrib-python eventlet==0.33.3 Flask==1.1.1 Flask-SocketIO==4.1.0 python-socketio==4.2.0 python-engineio==3.13.0 greenlet==1.0.0 gevent==21.1.2 gevent-websocket==0.10.1 Jinja2==3.0.3 itsdangerous==2.0.1 werkzeug==2.0.3 transforms3d

WORKDIR /home/projects

####### CREATE VIRTUAL ENVIRONMENTS #######
RUN apt-get update && apt-get install -y python3.8-venv && \
    echo "alias python=python3" >> ~/.bashrc && \
    python3 -m venv /opt/venv/donkey --system-site-packages

################ DEPTHAI ##################
WORKDIR /home/projects/
RUN git clone https://github.com/luxonis/depthai.git && \
    git clone https://github.com/luxonis/depthai-python.git && \
    cd depthai && \
    source /opt/venv/donkey/bin/activate && \
    curl -fL https://docs.luxonis.com/install_dependencies.sh | bash && \
    python3 install_requirements.py && \
    cd ../depthai-python/examples && \
    python3 install_requirements.py 

RUN echo "export OPENBLAS_CORETYPE=ARMV8" >> ~/.bashrc && \
    echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | tee /etc/udev/rules.d/80-movidius.rules

########### Install Groot ###########
WORKDIR /home

RUN apt-get update --fix-missing && \
    apt-get install -y --no-install-recommends ros-foxy-behaviortree-cpp-v3 libtool pkg-config build-essential autoconf automake qtbase5-dev libqt5svg5-dev libzmq3-dev libdw-dev && \
    rm -rf /var/lib/apt/lists/*

RUN git clone --recurse-submodules https://github.com/BehaviorTree/Groot.git
WORKDIR /home/Groot
RUN cmake -S . -B build && \
    cmake --build build

### Create f1tenth workspace
WORKDIR $WORKSPACE

RUN git clone -b foxy_devel https://github.com/Triton-AI/robocar.git && \
    mv robocar .. && \
    cd .. && \
    rm -rf $WORKSPACE && \
    mv robocar/ $WORKSPACE && \
    cd $WORKSPACE

# Global Planner Dependencies
RUN pip install trajectory_planning_helpers==0.78 scipy==1.7.3 matplotlib==3.5.1 casadi scikit-image==0.19.2 scikit-learn transforms3d

########### Clone Basic ROS 2 Packages ###########
RUN source source_ros2.sh && \
    vcs import < repos/common.repos && \
    vcs import < repos/lidar_utils.repos && \
    vcs import < repos/racer.repos && \ 
    sed -i '/#include <tf2_geometry_msgs/d' src/external/state_estimation/rf2o_laser_odometry/include/rf2o_laser_odometry/CLaserOdometry2DNode.hpp && \
    make gb_opt

### f1tenth gym && ros2 gym bridge ###
RUN make sim

# Copy and install AutoDRIVE Devkit (ROS 2 API)
RUN wget https://github.com/AutoDRIVE-Ecosystem/AutoDRIVE-F1TENTH-Sim-Racing/releases/download/2024-cdc/autodrive_devkit.zip && \
    unzip autodrive_devkit.zip -d ${WORKSPACE}src/external/ && \
    rm autodrive_devkit.zip

# COLCON BUILD ROS2 PACKAGES
RUN source source_ros2.sh && \
    make rosdep-install && \ 
    build_ros2

# Install ROS dependencies using rosdep
RUN source /opt/ros/foxy/setup.bash && \
    rosdep install --from-paths src --ignore-src -r -y

########### Build livox_sdk2, livox_ros_driver2 ###########
RUN source source_ros2.sh && \
    make livox-driver  

########### ADD CUSTOM FUNCTIONS ###########
WORKDIR /home/scripts/
COPY scripts/bashrc.sh ./bashrc.sh
RUN echo '. /home/scripts/bashrc.sh' >> /root/.bashrc

WORKDIR $WORKSPACE
RUN echo export WORKSPACE=$WORKSPACE >> /root/.bashrc
RUN /bin/bash -c 'echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc' && \
    /bin/bash -c 'source ~/.bashrc'

# Set entrypoint
EXPOSE 4567
ENTRYPOINT [ "/bin/bash"]