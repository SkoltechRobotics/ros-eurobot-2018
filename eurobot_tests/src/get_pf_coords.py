#!/usr/bin/env python
import rospy
import numpy as np
import tf2_ros
import tf_conversions
import sys


def get_coords():
    t = buffer1.lookup_transform('map', robot_name, rospy.Time(0))
    q = [t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w]
    yaw = tf_conversions.transformations.euler_from_quaternion(q)[2]
    return True, np.array([t.transform.translation.x * 1000, t.transform.translation.y * 1000, yaw])


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
    robot_name = sys.argv[2]
    print(robot_name)

    rospy.loginfo("start get data")
    coords_full = []
    rate = rospy.Rate(20)
    buffer1 = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(buffer1)
    time = int(sys.argv[1])
    for i in range(time * 20):
        rate.sleep()
        coords_full.append(get_coords())

    print calculate_main(np.array(coords_full))
