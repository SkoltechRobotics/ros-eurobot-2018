#!/bin/bash

source /opt/ros/kinetic/setup.bash
source ~/catkin_ws/devel/setup.bash

sudo rosnode kill -a
sudo killall rviz
sudo killall -9 rosmaster
sudo killall -9 roscore

roslaunch eurobot central_tracking_device.launch color:="$1" > ~/some_log.log&
rosrun eurobot_decision_maker bt_server.py &