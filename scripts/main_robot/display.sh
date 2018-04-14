#! /bin/bash
source /opt/ros/kinetic/setup.bash
source ~/catkin_ws/devel/setup.bash
export ROS_MASTER_URI=http://192.168.88.251:11311
#export ROS_HOSTNAME=192.168.2.102
export ROS_IP=192.168.88.250
export DISPLAY=:0.0
rosrun eurobot_gui display_node.py