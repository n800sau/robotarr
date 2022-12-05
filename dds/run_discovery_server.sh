#/bin/bash

cd `dirname $0`
source /opt/ros/humble/setup.bash
fastdds discovery --server-id 0 -b
