#!/bin/sh
export OPENCV_VERSION=4.8.0

mkdir /_temp && cd /_temp
wget -O opencv.zip https://github.com/opencv/opencv/archive/$OPENCV_VERSION.zip
unzip opencv.zip
rm opencv.zip
wget -O opencv_contrib.zip https://github.com/opencv/opencv_contrib/archive/$OPENCV_VERSION.zip
unzip opencv_contrib.zip
rm opencv_contrib.zip

mkdir /_temp/opencv-$OPENCV_VERSION/build
cd /_temp/opencv-$OPENCV_VERSION/build

cmake -D CMAKE_BUILD_TYPE=RELEASE \
    -D CMAKE_INSTALL_PREFIX=/usr/local \
    -D OPENCV_EXTRA_MODULES_PATH=/_temp/opencv_contrib-${OPENCV_VERSION}/modules \
    -D EIGEN_INCLUDE_PATH=/usr/include/eigen3 \
    -D WITH_OPENCL=OFF \
    -D WITH_CUDA=OFF \
    -D CUDA_ARCH_BIN=5.0 \
    -D CUDA_ARCH_PTX="" \
    -D WITH_CUDNN=OFF \
    -D WITH_CUBLAS=OFF \
    -D ENABLE_FAST_MATH=ON \
    -D CUDA_FAST_MATH=OFF \
    -D OPENCV_DNN_CUDA=OFF \
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
    -D WITH_PROTOBUF=ON \
    -D OPENCV_ENABLE_NONFREE=ON \
    -D INSTALL_C_EXAMPLES=OFF \
    -D INSTALL_PYTHON_EXAMPLES=OFF \
    -D PYTHON3_PACKAGES_PATH=/usr/lib/python3/dist-packages \
    -D OPENCV_GENERATE_PKGCONFIG=ON \
    -D BUILD_EXAMPLES=OFF ..

make -j4
make install
ldconfig
cd / && rm -rf /_temp


apt update

git clone --recurse-submodules https://github.com/osqp/osqp -b v0.6.2
cd osqp
mkdir build
cd build
cmake -G "Unix Makefiles" -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local ..
cmake --build .
cmake --build . --target install
cd ../..
rm -rf osqp

# install casadi. TODO(HaoruXue): Add back -DWITH_OPENCL=ON -DWITH_CLANG=ON
apt install -y gcc g++ gfortran git cmake liblapack-dev pkg-config --install-recommends
apt install -y --no-install-recommends coinor-libipopt-dev libslicot-dev
cd /opt && git clone https://github.com/casadi/casadi.git -b 3.6.4
cd casadi && mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release -DWITH_IPOPT=ON -DWITH_SLICOT=ON -DWITH_LAPACK=ON -DWITH_QPOASES=ON -DWITH_OSQP=ON .. && make -j$(nproc)
make install
cd /opt && rm -rf casadi

apt clean
rm -rf /var/lib/apt/lists/*
