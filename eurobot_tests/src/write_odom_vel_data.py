#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import numpy as np
import tf.transformations
import tf


def vel_callback(twist):
    global vel
    vel = [twist.linear.x, twist.linear.y, twist.angular.z]


def get_coords():
    (trans, rot) = listener.lookupTransform('/map', '/secondary_robot', rospy.Time(0))
    yaw = tf.transformations.euler_from_quaternion(rot)[2]
    return np.array([trans[0], trans[1], yaw])


if __name__ == '__main__':
    vel = None
    rospy.init_node("write_laser_data")
    rospy.Subscriber("/secondary_robot/cmd_vel", Twist, vel_callback)

    rospy.loginfo("start write data")
    odom_data_full = []
    vel_data_full = []
    rate = rospy.Rate(20)
    listener = tf.TransformListener()
    for i in range(400):
        rate.sleep()
        vel_data_full.append(vel)
        odom_data_full.append(get_coords())

    np.save("vel_cmd1.npy", np.array(vel_data_full))
    np.save("odom_coords1.npy", np.array(odom_data_full))

