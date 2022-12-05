#export ROS_DISCOVERY_SERVER=192.168.1.50:11811
export ROS_DOMAIN_ID=1
#export FASTRTPS_DEFAULT_PROFILES_FILE=fastdds_config_auxiliar_server.xml
export FASTRTPS_DEFAULT_PROFILES_FILE=super_client_configuration_file.xml
#export FASTRTPS_DEFAULT_PROFILES_FILE=initial_peers_multicast_avoidance.xml
#unset ROS_DISCOVERY_SERVER
#ros2 topic list
#ros2 node list
#ros2 node info /wheeler_3
# --spin 30 --no-daemon
#ros2 topic echo --no-daemon --once --spin 10 /tf
ros2 topic hz /tf
