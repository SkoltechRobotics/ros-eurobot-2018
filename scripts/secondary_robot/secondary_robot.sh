#! /bin/bash

source /opt/ros/kinetic/setup.bash
source ~/catkin_ws/devel/setup.bash
export ROS_MASTER_URI=http://192.168.88.245:11311
export ROS_IP=192.168.88.248
dt=`date -Iseconds`
while true; do
    echo "Start secondary robot script"
    roslaunch eurobot secondary_robot.launch >> /home/pi/log/secondary_robot_${dt}.log >> /home/pi/log/secondary_robot_error_${dt}.log
    sleep 1s
done
