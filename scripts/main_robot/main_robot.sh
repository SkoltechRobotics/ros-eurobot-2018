#! /bin/bash

source /opt/ros/kinetic/setup.bash
source ~/catkin_ws/devel/setup.bash
export ROS_MASTER_URI=http://192.168.88.251:11311
export ROS_IP=192.168.88.250
dt=$(date '+%d/%m/%Y %H:%M:%S');
while true; do
    roslaunch eurobot main_robot.launch > /home/pi/log/main_robot$dt.log
done