export ROS_MASTER_URI="http://rpi2:11311"
DIR="`dirname $PWD`"
. `realpath $DIR`/devel/setup.bash
echo $ROS_MASTER_URI
roslaunch main.launch &>run.log
