FROM ros:noetic
WORKDIR /home/n800s/ros1_ws
RUN useradd -ms /bin/bash -u 1100 n800s
RUN apt-get update
RUN apt-get install apt-utils
RUN apt-get -y install aptitude mc screen less
ENV ROS_MASTER_URI http://0.0.0.0:11311
#USER n800s
