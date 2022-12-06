#!/bin/bash
LOGPATH="`pwd`/make.log"
. /opt/ros/noetic/setup.bash
cd ..
catkin_make -j1 &>"${LOGPATH}"
echo $?
