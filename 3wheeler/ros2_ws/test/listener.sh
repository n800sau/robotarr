export ROS_DISCOVERY_SERVER=127.0.0.1:11811
export ROS_DOMAIN_ID=0
#export RMW_IMPLEMENTATION=rmw_fastrtps_dynamic_cpp
#export FASTRTPS_DEFAULT_PROFILES_FILE=super_client_configuration_file.xml
#ros2 run demo_nodes_cpp listener --ros-args --remap __node:=listener_discovery_server
ros2 run demo_nodes_cpp listener
