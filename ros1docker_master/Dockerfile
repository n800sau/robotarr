FROM ros:noetic
WORKDIR /home/n800s/ros1_ws
RUN useradd -ms /bin/bash -u 1100 n800s -G adm,sudo,audio,video,plugdev,netdev,n800s
RUN apt-get update
RUN apt-get install -y apt-utils
RUN apt-get install -y aptitude mc screen less iproute2 iputils-ping net-tools nmap git pyhton3-pip
RUN apt-get install -y ros-noetic-image-transport ros-noetic-cv-bridge ros-noetic-fkie-master-sync ros-noetic-fkie-master-discovery ros-noetic-robot-pose-ekf
ENV ROS_MASTER_URI http://0.0.0.0:11311
#USER n800s
