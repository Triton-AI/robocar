#!/bin/sh
apt update

# torchvision custom install
git clone --branch $TORCHVISION_INSTALL https://github.com/pytorch/vision torchvision  && \
cd torchvision && \
python3 setup.py install --user && \
rm -rf torchvision

git clone --recurse-submodules https://github.com/osqp/osqp -b v0.6.2
cd osqp
mkdir build
cd build
cmake -G "Unix Makefiles" -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local ..
cmake --build .
cmake --build . --target install
cd ../..
rm -rf osqp

apt install -y gcc g++ gfortran git cmake liblapack-dev pkg-config --install-recommends
apt install -y --no-install-recommends coinor-libipopt-dev libslicot-dev
cd /opt && git clone https://github.com/casadi/casadi.git -b 3.6.4
cd casadi && mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release -DWITH_IPOPT=ON -DWITH_SLICOT=ON -DWITH_LAPACK=ON -DWITH_QPOASES=ON -DWITH_OSQP=ON .. && make
make install
cd /opt && rm -rf casadi
apt clean
rm -rf /var/lib/apt/lists/*

# install pcl
git clone --recurse-submodules -b pcl-1.12.1 https://github.com/PointCloudLibrary/pcl.git
cd pcl && mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j2
sudo make -j2 install
cd ../..
rm -rf pcl

# Install Cmake
wget https://github.com/Kitware/CMake/archive/refs/tags/v3.22.1.zip
unzip v3.22.1.zip
cd CMake-3.22.1/
sudo ./bootstrap --prefix=/opt/cmake-install
sudo make
sudo make install
echo -e '\nexport PATH="/opt/cmake-install/bin:${PATH}"' >> ~/.bashrc
echo 'export CMAKE_PREFIX_PATH="/opt/cmake-install:${CMAKE_PREFIX_PATH}"' >> ~/.bashrc
source ~/.bashrc
cd /opt
rm -rf CMake-3.22.1
rm -rf v3.22.1.zip

wget -qO- https://raw.githubusercontent.com/luxonis/depthai-ros/main/install_dependencies.sh | sudo bash
cd /tmp
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd ./Livox-SDK2/
mkdir build && cd build
cmake .. && make -j4
sudo make install
cd /tmp
rm -rf Livox-SDK2