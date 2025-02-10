I added at the very bottom right after the LIVOX scripts, the scripts necessary for PointOneNav

On one terminal, ros2 launch ntrip_client ntrip_client_launch.py host:=polaris.pointonenav.com port:=2101 mountpoint:=POLARIS ntrip_version:=v2 authenticate:=true username:=gbUv1nO2 password:=zZLxzLpJ 
On another terminal, ros2 run fusion-engine-driver fusion_engine_ros_driver --ros-args -p connection_type:=tty -p tty_port:=/dev/ttyUSB1
