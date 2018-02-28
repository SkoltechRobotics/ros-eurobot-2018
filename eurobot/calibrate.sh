#! /bin/bash
echo "----------" >> calibration_data.txt
timeout 1s rosrun tf tf_echo /main_robot /world >> calibration_data.txt
echo "$1 $2 $3" >> calibration_data.txt 
timeout 1s rostopic pub /main_robot/stm_command std_msgs/String "data: 'SETSPEED 162 $1 $2 $3 0.2 0.2 0.5'"
