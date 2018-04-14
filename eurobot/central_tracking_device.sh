#!/bin/bash

source /opt/ros/kinetic/setup.bash
source ~/catkin_ws/devel/setup.bash

sudo rosnode kill -a
sudo killall rviz
sudo killall -9 rosmaster
sudo killall -9 roscore

roslaunch eurobot central_tracking_device.launch color:=$1
#sleep 2 ssh pi@192.168.88.248 "~/catkin_ws/src/ros-eurobot-2018/scripts/main_robot/main_robot.sh" > main_robot.log&
#sleep 2 ssh pi@192.168.88.250 "~/catkin_ws/src/ros-eurobot-2018/scripts/secondary_robot/secondary_robot.sh" > secondary_robot.log&
