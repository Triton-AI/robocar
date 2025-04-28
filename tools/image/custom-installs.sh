#!/bin/sh
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
cmake -DCMAKE_BUILD_TYPE=Release -DWITH_IPOPT=ON -DWITH_SLICOT=ON -DWITH_LAPACK=ON -DWITH_QPOASES=ON -DWITH_OSQP=ON .. && make
make install
cd /opt && rm -rf casadi

apt clean
rm -rf /var/lib/apt/lists/*
