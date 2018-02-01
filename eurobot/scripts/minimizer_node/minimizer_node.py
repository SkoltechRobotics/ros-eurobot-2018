#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
import numpy as np
import scipy.optimize

BEACONS = np.array([[-70, 30], [-70, 2000 - 30], [3000 + 70, 1000]])
R = 50


def cvt_local2global_1(local_point, sc_point):
    point = np.zeros(3)
    x, y, a = local_point
    x_g, y_g, a_g = sc_point
    point[0] = x * np.cos(a_g) - y * np.sin(a_g) + x_g
    point[1] = x * np.sin(a_g) + y * np.cos(a_g) + y_g
    point[2] = a + a_g
    return point


def cvt_global2local_1(global_point, sc_point):
    point = np.zeros(3)
    x, y, a = global_point
    x_g, y_g, a_g = sc_point
    point[0] = x * np.cos(a_g) + y * np.sin(a_g) - x_g * np.cos(a_g) - y_g * np.sin(a_g)
    point[1] = -x * np.sin(a_g) + y * np.cos(a_g) + x_g * np.sin(a_g) - y_g * np.cos(a_g)
    point[2] = a - a_g
    return point


def cvt_local2global(local_point, sc_point):
    point = np.zeros((3, len(local_point)))
    x, y, a = local_point.T
    x_global, y_global, a_global = sc_point
    point[0] = x * np.cos(a_global) - y * np.sin(a_global) + x_global
    point[1] = x * np.sin(a_global) + y * np.cos(a_global) + y_global
    point[2] = a + a_global
    return point.T


class Minimizer(object):
    def __init__(self, x, y, angle, min_i, max_d, alpha=0.05):
        self.point = np.array([x, y, angle])
        self.min_i = min_i
        self.max_d = max_d
        self.stm_point = self.point
        self.smooth = True
        self.coords_in_stm = np.array([0, 0, 0])
        self.alpha = alpha

    def find_point(self, scan):
        # The most probable point
        point = cvt_local2global_1(self.coords_in_stm, self.stm_point)

        # Use scans
        angles = np.linspace(0, 270, len(scan)) / 180 * np.pi - np.pi / 4
        ranges = scan.T[0]
        intens = scan.T[1]

        cond = (ranges < self.max_d) * (intens > self.min_i)
        x = (ranges * np.cos(angles))[cond]
        y = (ranges * np.sin(angles))[cond]
        points = np.zeros((len(x), 3))
        points[:, 0] = x
        points[:, 1] = y

        apr_points = cvt_local2global(points, point)[:, 0:2]
        beacons_len = np.sum((BEACONS[np.newaxis, :, :] - apr_points[:, np.newaxis, :]
                              ) ** 2, axis=2) ** 0.5
        num_beacons = np.argmin(beacons_len, axis=1)

        bounds = (point - np.array([20, 20, np.pi / 90])), point + np.array([20, 20, np.pi / 90])

        res = scipy.optimize.least_squares(self.fun, point, loss="linear",
                                           bounds=bounds,
                                           args=[points, num_beacons], ftol=1e-3)

        # Smoothing
        new_point = cvt_global2local_1(res.x, self.stm_point)
        self.coords_in_stm = self.coords_in_stm + self.alpha * (new_point - self.coords_in_stm)
        self.point = cvt_local2global_1(self.coords_in_stm, self.stm_point)
        # print "point", point
        # print "res.x", res.x
        # print "stm_point", self.stm_point
        # print "coords_in_stm", self.coords_in_stm
        # print "self.point", self.point
        # print "--------------"
        return self.point

    @staticmethod
    def fun(point, points, num_beacons):
        beacons = BEACONS[num_beacons]
        points = cvt_local2global(points, point)[:, 0:2]
        r = np.sum((beacons - points) ** 2, axis=1) ** 0.5 - R
        return r

    def set_stm_coords(self, point):
        self.stm_point = point


def scan_callback(scan):
    lidar_data = np.array([np.array(scan.ranges) * 1000, scan.intensities]).T
    global minimizer
    global pub
    pub.publish(' '.join(map(str, minimizer.find_point(lidar_data))))


def stm_callback(data):
    z = map(float, data.data.split())
    minimizer.set_stm_coords(z)


if __name__ == '__main__':
    try:
        # create a PF object with params from ROS
        in_x = rospy.get_param("start_x")
        in_y = rospy.get_param("start_y")
        in_angle = rospy.get_param("start_a")
        max_itens = rospy.get_param("particle_filter/max_itens")
        max_dist = rospy.get_param("particle_filter/max_dist")
        minimizer = Minimizer(x=in_x, y=in_y, angle=in_angle, min_i=max_itens, max_d=max_dist)

        # ROS entities

        rospy.init_node('minimizer_node', anonymous=True)
        pub = rospy.Publisher('minimizer/coordinates', String, queue_size=1)
        rospy.Subscriber("scan", LaserScan, scan_callback, queue_size=1)  # lidar data
        rospy.Subscriber("stm/coordinates", String, stm_callback, queue_size=1)

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
