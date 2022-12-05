#export ROS_DISCOVERY_SERVER=192.168.1.50:11811
export ROS_DOMAIN_ID=1
#export FASTRTPS_DEFAULT_PROFILES_FILE=initial_peers_multicast_avoidance.xml
export FASTRTPS_DEFAULT_PROFILES_FILE=super_client_configuration_file.xml
#export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export RMW_IMPLEMENTATION=rmw_fastrtps_dynamic_cpp
#unset ROS_DISCOVERY_SERVER
#ros2 run demo_nodes_cpp talker --ros-args --remap __node:=talker_discovery_server
ros2 run demo_nodes_cpp talker
