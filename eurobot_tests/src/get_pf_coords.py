#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
import numpy as np
import tf.transformations
import tf
import sys


def get_coords():
    (trans, rot) = listener.lookupTransform('/map', '/%s' % robot_name, rospy.Time(0))
    yaw = tf.transformations.euler_from_quaternion(rot)[2]
    return np.array([trans[0], trans[1], yaw])


def calculate_main(points):
    x = np.mean(points[:, 0])
    y = np.mean(points[:, 1])
    zero_elem = points[0, 2]
    # this helps if particles angles are close to 0 or 2*pi
    temporary = ((points[:, 2] - zero_elem + np.pi) % (2.0 * np.pi)) + zero_elem - np.pi
    angle = np.mean(temporary) % (2.0 * np.pi)
    return np.array((x, y, angle))


if __name__ == '__main__':
    lidar_data = None
    rospy.init_node("get_coords")
    robot_name = sys.argv[3]

    rospy.loginfo("start get data")
    coords_full = []
    rate = rospy.Rate(20)
    listener = tf.TransformListener()
    time = int(sys.argv[1])
    for i in range(time * 20):
        rate.sleep()
        coords_full.append(get_coords())

    print calculate_main(np.array(coords_full))
    