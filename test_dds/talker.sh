export ROS_DISCOVERY_SERVER=192.168.1.50:11811
export ROS_DOMAIN_ID=0
#export FASTRTPS_DEFAULT_PROFILES_FILE=super_client_configuration_file.xml
#export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
#export RMW_IMPLEMENTATION=rmw_fastrtps_dynamic_cpp
#ros2 run demo_nodes_cpp talker --ros-args --remap __node:=talker_discovery_server
ros2 run demo_nodes_cpp talker
