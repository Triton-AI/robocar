FROM nvcr.io/nvidia/l4t-base:35.4.1 as base
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ARG DEBIAN_FRONTEND=noninteractive

######### JETPACK FOR JETSON #########
RUN apt-get update && apt-get install -y --no-install-recommends \
    nvidia-cuda-dev \
    nvidia-cudnn8-dev \
    && rm -rf /var/lib/apt/lists/* \
    && apt-get clean

# Install nvidia-tensorrt-dev for TensorRT developer packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    nvidia-tensorrt-dev \
    && rm -rf /var/lib/apt/lists/* \
    && apt-get clean

# Install nvidia-vpi-dev for VPI developer packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    nvidia-vpi-dev \
    && rm -rf /var/lib/apt/lists/* \
    && apt-get clean

# Install Multimedia API samples & libs
RUN apt-get update && apt-get download nvidia-l4t-jetson-multimedia-api \
    && dpkg-deb -R ./nvidia-l4t-jetson-multimedia-api_*_arm64.deb ./mm-api \
    && cp -r ./mm-api/usr/src/jetson_multimedia_api /usr/src/jetson_multimedia_api \
    && ./mm-api/DEBIAN/postinst \
    && rm -rf ./nvidia-l4t-jetson-multimedia-api_*_arm64.deb ./mm-api

# Setup environment variables
ENV CUDA_HOME="/usr/local/cuda"
ENV PATH="/usr/local/cuda/bin:${PATH}"
ENV LD_LIBRARY_PATH="/usr/local/cuda/lib64:${LD_LIBRARY_PATH}"

#Update libraries
RUN ldconfig

FROM base AS cv2

ARG DEBIAN_FRONTEND=noninteractive
ARG OPENCV_VERSION=4.9.0

RUN apt-get update && apt-get upgrade -y &&\
    # Install build tools, build dependencies and python
    apt-get install -y \
	python3-pip \
        build-essential \
        cmake \
        git \
        wget \
        unzip \
        yasm \
        libopenblas-dev \
        curl \
        sudo \
        pkg-config \
        libswscale-dev \
        libtbb2 \
        libtbb-dev \
        libjpeg-dev \
        libpng-dev \
        libtiff-dev \
        libavformat-dev \
        libpq-dev \
        libxine2-dev \
        libglew-dev \
        libtiff5-dev \
        zlib1g-dev \
        libjpeg-dev \
        libavcodec-dev \
        libavformat-dev \
        libavutil-dev \
        libpostproc-dev \
        libswscale-dev \
        libeigen3-dev \
        libtbb-dev \
        libgtk2.0-dev \
        pkg-config \
        python3-dev \
        python3-numpy \
    && rm -rf /var/lib/apt/lists/*

RUN cd /opt/ &&\
    # Download and unzip OpenCV and opencv_contrib and delte zip files
    wget https://github.com/opencv/opencv/archive/$OPENCV_VERSION.zip &&\
    unzip $OPENCV_VERSION.zip &&\
    rm $OPENCV_VERSION.zip &&\
    wget https://github.com/opencv/opencv_contrib/archive/$OPENCV_VERSION.zip &&\
    unzip ${OPENCV_VERSION}.zip &&\
    rm ${OPENCV_VERSION}.zip &&\
    # Create build folder and switch to it
    mkdir /opt/opencv-${OPENCV_VERSION}/build && cd /opt/opencv-${OPENCV_VERSION}/build && \
    # Cmake configure
    cmake -D CMAKE_BUILD_TYPE=RELEASE \
        -D OPENCV_EXTRA_MODULES_PATH=/opt/opencv_contrib-${OPENCV_VERSION}/modules \
        -D EIGEN_INCLUDE_PATH=/usr/include/eigen3 \
        -D WITH_OPENCL=OFF \
        -D WITH_CUDA=ON \
        -D CUDA_ARCH_BIN= 5.0,7.2,8.7 \
        -D CUDA_ARCH_PTX="" \
        -D WITH_CUDNN=ON \
        -D WITH_CUBLAS=ON \
        -D ENABLE_FAST_MATH=ON \
        -D CUDA_FAST_MATH=ON \
        -D OPENCV_DNN_CUDA=ON \
        -D ENABLE_NEON=ON \
        -D WITH_QT=OFF \
        -D WITH_OPENMP=ON \
        -D BUILD_TIFF=ON \
        -D WITH_FFMPEG=ON \
        -D WITH_GSTREAMER=ON \
        -D WITH_TBB=ON \
        -D BUILD_TBB=ON \
        -D BUILD_TESTS=OFF \
        -D WITH_EIGEN=ON \
        -D WITH_V4L=ON \
        -D WITH_LIBV4L=ON \
        -D OPENCV_ENABLE_NONFREE=ON \
        -D INSTALL_C_EXAMPLES=OFF \
        -D INSTALL_PYTHON_EXAMPLES=OFF \
        -D BUILD_NEW_PYTHON_SUPPORT=ON \
        -D BUILD_opencv_python3=TRUE \
        -D OPENCV_GENERATE_PKGCONFIG=ON \
        -D BUILD_EXAMPLES=OFF \
        .. &&\
    # Make
    make -j"$(nproc)" && \
    # Install to /usr/local/lib
    make install && \
    ldconfig && \
    # # Remove OpenCV sources and build folder
    rm -rf /opt/opencv-${OPENCV_VERSION} && rm -rf /opt/opencv_contrib-${OPENCV_VERSION}

RUN echo 'export PYTHONPATH=/usr/local/lib/python3.8/site-packages/:$PYTHONPATH' >> ~/.bashrc

FROM cv2 as dl_gpu

RUN python3 -m pip install --upgrade pip 

ARG JP_VERSION=512
ARG TF_VERSION=2.12.0
ARG NV_VERSION=23.06
RUN pip3 install --extra-index-url https://developer.download.nvidia.com/compute/redist/jp/v$JP_VERSION tensorflow==$TF_VERSION+nv$NV_VERSION

######### PyTorch for Jetpack #########
ARG TORCH_INSTALL=https://developer.download.nvidia.com/compute/redist/jp/v512/pytorch/torch-2.1.0a0+41361538.nv23.06-cp38-cp38-linux_aarch64.whl
RUN python3 -m pip install --no-cache $TORCH_INSTALL

######### TorchVision for Jetpack #########
ARG TORCHVISION_INSTALL=release/0.16
RUN git clone --branch $TORCHVISION_INSTALL https://github.com/pytorch/vision torchvision  && \
    cd torchvision && \
    python3 setup.py install --user && \
    rm -rf /torchvision

FROM dl_gpu as utils

# install useful packages
RUN echo "wireshark-common wireshark-common/install-setuid boolean true" | debconf-set-selections && \
    apt-get update && \
    apt-get install -y --no-install-recommends \
        python3-dev \
        nano \
        iputils-ping \
        x11-apps \
        nautilus \
        firefox \
        git-all \
        cheese \
        vim \
        wireshark \
        python3-argcomplete \
        jstest-gtk \
        joystick \
        gedit \
        gedit-plugin-multi-edit \
        gedit-plugins \
        python3-tk \
        python3.8-venv \
    && rm -rf /var/lib/apt/lists/* \
    && apt-get clean

FROM utils as ros

# --------------------------------------------------------------------------------------------------------------------------------------------------
# Install ROS2 Foxy 
# --------------------------------------------------------------------------------------------------------------------------------------------------
RUN apt update &&\
    apt install software-properties-common -y && \
    add-apt-repository universe && \
    apt update && sudo apt install curl -y && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

WORKDIR /

# --------------------------------------------------------------------------------------------------------------------------------------------------
# intel debian packages
# --------------------------------------------------------------------------------------------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    libssl-dev \
    libusb-1.0-0-dev \
    pkg-config \
    libgtk-3-dev \
    libglfw3-dev \
    libgl1-mesa-dev \
    libglu1-mesa-dev \
    libssl-dev \
    libusb-1.0-0-dev \
    pkg-config \
    libgtk-3-dev \
    libglfw3-dev \
    libgl1-mesa-dev \
    libglu1-mesa-dev \
    build-essential \
    libgtk-3-dev \
    libgl1-mesa-dev \
    libglu1-mesa-dev \
    build-essential \
    libgtk-3-dev


# --------------------------------------------------------------------------------------------------------------------------------------------------
# install ros2 packages
# --------------------------------------------------------------------------------------------------------------------------------------------------
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
    && rm -rf /var/lib/apt/lists/*

# --------------------------------------------------------------------------------------------------------------------------------------------------
# install useful packages
# --------------------------------------------------------------------------------------------------------------------------------------------------
WORKDIR /
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    python3-pip \
    python3-dev \
    python3-argcomplete \
    cmake \
    nano \
    iputils-ping \
    x11-apps \
    nautilus \
    usbutils \
    vim \
    tmux \
    tmuxp \
    htop \
    network-manager \
    firefox \
    git-all \
    cheese \
    jstest-gtk \
    joystick \
    gedit \
    gedit-plugin-multi-edit \
    gedit-plugins \
    python3-tk \
    python3-rosdep \
    python3-vcstool \
    ros-dev-tools

RUN wget -qO- https://raw.githubusercontent.com/luxonis/depthai-ros/main/install_dependencies.sh | sudo bash

RUN mkdir -p ~/.ssh \
   && ssh-keyscan github.com >> ~/.ssh/known_hosts
   && ssh-keyscan gitlab.com >> ~/.ssh/known_hosts

RUN export ROS_DISTRO=foxy
RUN echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc

RUN rosdep init

WORKDIR /home/jetson/

CMD ["/bin/bash", "-l", "-c", "exec /bin/bash"]
