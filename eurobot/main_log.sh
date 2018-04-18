#! /bin/bash

file=`ssh pi@main_robot ls /home/pi/log -Art | tail -n 1`
echo $file
ssh pi@main_robot tail -f $file
