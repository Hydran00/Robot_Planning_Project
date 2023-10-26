#!/bin/bash
source "/opt/ros/humble/setup.bash"
source "/root/ros2_ws/install/setup.bash"

fastdds discovery --server-id 0 &

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=3
export ROS_DISCOVERY_SERVER="10.196.80.115:11811;10.196.80.125:11811"

route add -net 224.0.0.0 netmask 240.0.0.0 dev enp1s0
ifconfig enp1s0 multicast

# export FASTRTPS_DEFAULT_PROFILES_FILE=super_client_configuration_file.xml
ros2 daemon stop
ros2 daemon start
