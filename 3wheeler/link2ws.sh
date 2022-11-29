DIRNAME=`dirname $0`
#echo $DIRNAME
ROS2_PACK_PATH=`realpath ${DIRNAME}`/ros2_ws/wheeler_3
#echo $ROS2_PACK_PATH
ln -s "${ROS2_PACK_PATH}" ~/ros2_ws/src/wheeler_3
