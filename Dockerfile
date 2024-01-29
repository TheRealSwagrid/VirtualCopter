FROM ros:noetic
SHELL ["/bin/bash", "-c"]

ENV semantix_port=7500
ENV xmlrpc_port=45100
ENV tcpros_port=45101
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_IP=127.0.0.1
ENV ROS_MASTER_URI=http://127.0.0.1:11311

# ROS-Noetic Setup
RUN sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt-get update && apt-get install -y curl vim
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

RUN sudo apt-get update
RUN apt-get update && apt-get install -y python3-rosinstall python3-rosinstall-generator python3-wstool  \
    build-essential python3-rosdep python3-catkin-tools ros-noetic-vrpn-client-ros python-is-python3 python3-pip ros-noetic-tf

# Add Files
ADD ros_ws /ros_ws
COPY protocols /etc
COPY VirtualCopter.py /ros_ws/src/copterhandler/src
COPY AbstractVirtualCapability.py ros_ws/src/copterhandler/src

RUN  vi +':wq ++ff=unix' /ros_ws/src/copterhandler/src/VirtualCopter.py
RUN  vi +':wq ++ff=unix' /ros_ws/src/copterhandler/src/AbstractVirtualCapability.py
RUN  vi +':wq ++ff=unix' /ros_ws/src/copterhandler/src/copter_handler.py

# Build Ros-Pkg and build
RUN cd /ros_ws && source /opt/ros/noetic/setup.bash && catkin_make
ENTRYPOINT ["/ros_entrypoint.sh"]

CMD source /ros_ws/devel/setup.bash && roslaunch copterhandler copterhandler.launch semantix_port:=${semantix_port}
