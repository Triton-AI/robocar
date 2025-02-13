# For Python packages
echo export PATH="${PATH:+:${PATH}}/usr/local/lib/python3.8/dist-packages" >> /etc/bash.bashrc
# For ROS2
echo "source /opt/ros/${ROS_SOURCE}/setup.bash" >> /etc/bash.bashrc
echo export "RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> /etc/bash.bashrc
echo export "CYCLONEDDS_URI=file:///etc/cyclone/cyclonedds.xml" >> /etc/bash.bashrc
echo export "CYCLONE_INCLUDE=/opt/ros/${ROS_SOURCE}/include" >> /etc/bash.bashrc
echo export "CYCLONE_LIB=/opt/ros/${ROS_SOURCE}/lib/" >> /etc/bash.bashrc
echo export "ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST" >> /etc/bash.bashrc
echo export "ROS_STATIC_PEERS=" >> /etc/bash.bashrc
# For Casadi
echo export LD_LIBRARY_PATH="${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}/usr/local/lib" >> /etc/bash.bashrc
# For CMake
echo -e '\nexport PATH="/opt/cmake-install/bin:${PATH}"' >> /etc/bash.bashrc
echo 'export CMAKE_PREFIX_PATH="/opt/cmake-install:${CMAKE_PREFIX_PATH}"' >> /etc/bash.bashrc
