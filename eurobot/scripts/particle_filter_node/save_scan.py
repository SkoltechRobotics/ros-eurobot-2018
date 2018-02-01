#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
import numpy as np


def scan_callback(scan):
    global f
    if f:
        lidar_data = np.array([np.array(scan.ranges) * 1000, scan.intensities]).T
        np.save("/home/mikhail/Documents/Eurobot2018/Localisation/scan.npy", lidar_data)
        f = False


if __name__ == '__main__':
    try:
        f = True
        rospy.init_node('save_scan', anonymous=True)

        rospy.Subscriber("/main_robot/scan", LaserScan, scan_callback, queue_size=1)  # lidar data
        rospy.sleep(0.5)
    except rospy.ROSInterruptException:
        pass
