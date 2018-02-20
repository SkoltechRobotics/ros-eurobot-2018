# ros-eurobot-2018

`How to run this:` <br/>

`$ cd /path/to/catkin_ws/src` <br/>
`$ git clone https://github.com/SkoltechRobotics/ros-eurobot-2018.git` <br/>
`$ cd /path/to/catkin_ws` <br/>
`$ catkin_make` <br/>
`$ source devel/setup.bash` <br/>
`$ rospack find eurobot_decision_maker`

`Might require:` <br/>
`$ sudo apt-get install ros-kinetic-moveit ros-kinetic-rviz-visual-tools ros-kinetic-move-base ros-kinetic-map-server`

`How to use LIDAR:` <br/>
`We use urg_node, a ROS package that works with Hokuyo lidars` <br/>

`How to work with lidar:` <br/>
`1. Connect lidar via Ethernet.` <br/>
`2. Set on computer IP 192.168.0.XX, where XX != 10 (192.168.0.10 is the IP of the lidar):` <br/>
`$ sudo ifconfig "interface_name" 192.168.0.XX netmask 255.255.255.0` <br/>
`3. Run urg_node with the command:` <br/>
`$ rosrun urg_node urg_node _ip_address:=192.168.0.10` <br/>
`4. Done. Now there is a topic "/scan"` <br/>
`If it does not work, make sure you can ping lidar.` <br/>

`sensor_msgs/LaserScan Message contains timing of the scan. If the lidar is moving during scanning, it is possible to make corrections of the scanned "slice" (using odometry data). However the error related to lidar movement is probably really small. If so, we can just not care about it.` <br/>

`How to work on PC with lidar, connected to RPI:` <br/>
`1. For each computer other than master computer (XXX - master computer IP):` <br/>
`$ export ROS_MASTER_URI=http://192.168.2.XXX:11311` <br/>
`2. For each computer (XXX - this computer IP):` <br/>
`$ export ROS_IP=192.168.2.XXX` <br/>
`3. Now everything should work properly. For further instructions comment to "ROS nodes for electronics".` <br/>
