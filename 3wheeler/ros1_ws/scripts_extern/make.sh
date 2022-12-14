#!/bin/bash
. /opt/ros/noetic/setup.bash
cd ..
catkin_make install -j1 &>make.log
echo $?
