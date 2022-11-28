ros2 topic pub --once /cmd_vel geometry_msgs/Twist 'linear: {x: 0}'
sleep 1
ros2 topic pub --once /cmd_vel geometry_msgs/Twist 'linear: {x: -0.5}'
sleep 1
ros2 topic pub --once /cmd_vel geometry_msgs/Twist 'linear: {x: 0}'
sleep 1
ros2 topic pub --once /cmd_vel geometry_msgs/Twist '{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.0}}'
sleep 1
ros2 topic pub --once /cmd_vel geometry_msgs/Twist 'linear: {x: 0}'
