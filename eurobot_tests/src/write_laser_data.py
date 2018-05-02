#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
import numpy as np
import tf.transformations
import tf


def scan_callback(scan):
    global lidar_data
    lidar_data = np.array([np.array(scan.ranges) * 1000, scan.intensities]).T


def get_coords():
    (trans, rot) = listener.lookupTransform('/secondary_robot_odom', '/secondary_robot', rospy.Time(0))
    yaw = tf.transformations.euler_from_quaternion(rot)[2]
    return np.array([trans[0], trans[1], yaw])


if __name__ == '__main__':
    lidar_data = None
    rospy.init_node("write_laser_data")
    rospy.Subscriber("/secondary_robot/scan", LaserScan, scan_callback)

    rospy.loginfo("start write data")
    lidar_data_full = []
    odom_data_full = []
    rate = rospy.Rate(20)
    listener = tf.TransformListener()
    for i in range(400):
        rate.sleep()
        lidar_data_full.append(lidar_data)
        odom_data_full.append(get_coords())
#        rospy.loginfo("add line %d" % i)

    np.save("laser_scans5.npy", np.array(lidar_data_full))
    np.save("odom_coords5.npy", np.array(odom_data_full))
