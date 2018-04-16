#! /bin/bash

source /opt/ros/kinetic/setup.bash
source ~/catkin_ws/devel/setup.bash
export ROS_MASTER_URI=http://192.168.88.251:11311
export ROS_IP=192.168.88.250
dt=`date -Iseconds`
echo "Start main robot script"
while true; do
    echo "Try main robot launch"
    roslaunch eurobot main_robot.launch >> /home/pi/log/main_robot_${dt}.log >> /home/pi/log/main_robot_error_${dt}.log
done
