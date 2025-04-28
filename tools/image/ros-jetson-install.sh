apt update && \
    DEBIAN_FRONTEND=noninteractive apt install -y --no-install-recommends \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-vcstool \
    libignition-math6-dev \
    python3-rosdep && \
    apt clean && \
    rm -rf /var/lib/apt/lists/*

colcon mixin add default \
    https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml && \
    colcon mixin update && \
    colcon metadata add default \
    https://raw.githubusercontent.com/colcon/colcon-metadata-repository/master/index.yaml && \
    colcon metadata update
