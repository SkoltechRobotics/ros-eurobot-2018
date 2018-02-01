#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import numpy as np


def cvt_local2global(local_point, sc_point):
    point = np.zeros(3)
    x, y, a = local_point
    x_g, y_g, a_g = sc_point
    point[0] = x * np.cos(a_g) - y * np.sin(a_g) + x_g
    point[1] = x * np.sin(a_g) + y * np.cos(a_g) + y_g
    point[2] = a + a_g
    return point


def cvt_global2local(global_point, sc_point):
    point = np.zeros(3)
    x, y, a = global_point
    x_g, y_g, a_g = sc_point
    point[0] = x * np.cos(a_g) + y * np.sin(a_g) - x_g * np.cos(a_g) - y_g * np.sin(a_g)
    point[1] = -x * np.sin(a_g) + y * np.cos(a_g) + x_g * np.sin(a_g) - y_g * np.cos(a_g)
    point[2] = a - a_g
    return point


class Smoother(object):
    def __init__(self, global_p, a=0.03):
        self.global_p = global_p
        self.local_p = np.array([0, 0, 0])
        self.a = a

    def set_global_point(self, global_p):
        self.global_p = global_p

    def smooth(self, p):
        new_local_p = cvt_global2local(p, self.global_p)
        self.local_p = self.local_p + self.a * (new_local_p - self.local_p)
        return cvt_local2global(self.local_p, self.global_p)


def stm_coords_input_callback(data):
    z = map(float, data.data.split())
    sm.set_global_point(z)


def coords_input_callback(data):
    z = map(float, data.data.split())
    x = sm.smooth(z)
    pub.publish(' '.join(map(str, x)))


if __name__ == '__main__':
    try:
        # create a KF object with params from ROS 
        initial_coords = np.zeros(3)
        initial_coords[0] = rospy.get_param("start_x")
        initial_coords[1] = rospy.get_param("start_y")
        initial_coords[2] = rospy.get_param("start_a")
        sm = Smoother(initial_coords)

        # ROS entities
        rospy.init_node('smoother_filter', anonymous=True)
        rospy.Subscriber("particle_filter/coordinates", String, coords_input_callback, queue_size=1)
        rospy.Subscriber("stm/coordinates", String, stm_coords_input_callback, queue_size=1)
        pub = rospy.Publisher('smoother/coordinates', String, queue_size=1)

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
