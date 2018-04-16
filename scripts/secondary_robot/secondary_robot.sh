#! /bin/bash

source /opt/ros/kinetic/setup.bash
source ~/catkin_ws/devel/setup.bash
export ROS_MASTER_URI=http://192.168.88.252:11311
export ROS_IP=192.168.88.250
dt=`date -Iseconds`
echo "Start secondary robot script"
while true; do
    echo "Try secondary robot launch"
    roslaunch eurobot secondary_robot.launch >> /home/pi/log/secondary_robot_${dt}.log >> /home/pi/log/secondary_robot_error_${dt}.log
done