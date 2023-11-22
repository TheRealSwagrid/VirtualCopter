#!/usr/bin/env bash

port="$1"
cp VirtualCopter.py /ros_ws/src/copterhandler/src
cp AbstractVirtualCapability.py ros_ws/src/copterhandler/src
cd /ros_ws && source /opt/ros/noetic/setup.bash && catkin_make
source /ros_ws/devel/setup.bash && roslaunch copterhandler copterhandler.launch semantix_port:="$port"
