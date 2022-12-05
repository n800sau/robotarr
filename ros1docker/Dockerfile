FROM ros:noetic-ros-base-focal
WORKDIR /home/n800s/ros1_ws
RUN useradd -ms /bin/bash -u 1100 n800s
RUN apt-get update
RUN apt-get install apt-utils
RUN apt-get -y install aptitude mc screen less
RUN apt-get install -y ros-noetic-image-transport ros-noetic-cv-bridge
ENV ROS_MASTER_URI http://192.168.1.50:11311
#USER n800s
