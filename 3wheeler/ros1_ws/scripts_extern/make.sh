#!/bin/bash
. /opt/ros/noetic/setup.bash
cd ..
catkin_make -j1 &>make.log
echo $?
