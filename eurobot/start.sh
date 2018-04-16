#!/bin/bash

source /opt/ros/kinetic/setup.bash
source ~/catkin_ws/devel/setup.bash

sudo rosnode kill -a
sudo killall rviz
sudo killall -9 rosmaster
sudo killall -9 roscore

roslaunch eurobot central_tracking_device.launch color:="$1" > ~/some_log.log&
# ssh pi@192.168.88.248 "/home/pi/catkin_ws/src/ros-eurobot-2018/scripts/secondary_robot/secondary_robot.sh" &
# ssh pi@192.168.88.250 "/home/pi/catkin_ws/src/ros-eurobot-2018/scripts/main_robot/main_robot.sh" &
rosrun eurobot_decision_maker bt_server.py &
./display.sh &