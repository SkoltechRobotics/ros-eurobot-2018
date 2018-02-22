#!/bin/bash

sudo rosnode kill -a
sudo killall rviz
sudo killall -9 rosmaster
sudo killall -9 roscore

roslaunch eurobot_sim demo.launch
