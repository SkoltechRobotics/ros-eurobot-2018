#! /bin/bash

source /opt/ros/kinetic/setup.bash
source ~/catkin_ws/devel/setup.bash
export ROS_MASTER_URI=http://192.168.88.245:11311
export ROS_IP=192.168.88.248
dt=`date -Iseconds`
echo "Start secondary robot script" >> /home/pi/log/secondary_robot_${dt}.log
while true; do
    echo "Start secondary robot script" /home/pi/log/secondary_robot_${dt}.log
    echo `date -Iseconds` >> /home/pi/log/secondary_robot_${dt}.log
    roslaunch eurobot secondary_robot.launch &>> /home/pi/log/secondary_robot_${dt}.log
    sleep 1s
done
