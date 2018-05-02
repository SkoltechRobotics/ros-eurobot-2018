#!/usr/bin/env python
import rospy
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import LaserScan
import numpy as np
from npParticle import ParticleFilter, cvt_global2local, cvt_local2global, find_src
import tf2_ros
import tf_conversions
import time

PF_RATE = 20

PF_PARAMS = {"sense_noise": 2,
             "distance_noise": 8,
             "angle_noise": 0.1,
             "min_intens": 3000,
             "max_dist": 3700,
             "back_side_cost": 10,
             "k_angle": 200,
             "particles_num": 500,
             "beac_dist_thresh": 50,
             "k_mult": 0}


class PFNode(object):
    # noinspection PyTypeChecker
    def __init__(self):
        # Init params
        rospy.Subscriber("scan", LaserScan, self.scan_callback, queue_size=1)
        self.color = rospy.get_param("/field/color")
        self.robot_name = rospy.get_param("robot_name")
        self.lidar_point = np.array([rospy.get_param("lidar_x"), rospy.get_param("lidar_y"),
                                     rospy.get_param("lidar_a")])

        self.scan = None
        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer)
        self.br = tf2_ros.TransformBroadcaster()

        rospy.sleep(1)

        robot_odom_point = self.get_odom()
        lidar_odom_point = cvt_local2global(self.lidar_point, robot_odom_point)
        self.prev_lidar_odom_point = lidar_odom_point
        x, y, a = lidar_odom_point
        self.pf = ParticleFilter(color=self.color, start_x=x, start_y=y, start_angle=a, **PF_PARAMS)
        self.last_odom = np.zeros(3)
        self.alpha = 0.15

        rospy.Timer(rospy.Duration(1. / PF_RATE), self.localisation)

    def scan_callback(self, scan):
        self.scan = np.array([np.array(scan.ranges) * 1000, scan.intensities]).T

    def get_odom_frame(self, robot_pf_point, robot_odom_point):
        odom = find_src(robot_pf_point, robot_odom_point)
        self.last_odom[:2] = (1 - self.alpha) * self.last_odom[:2] + self.alpha * odom[:2]
        self.last_odom[2] = odom[2]
        return self.last_odom

    # noinspection PyUnusedLocal
    def localisation(self, event):
        time1 = time.time()
        robot_odom_point = self.get_odom()
        time2 = time.time()
        lidar_odom_point = cvt_local2global(self.lidar_point, robot_odom_point)
        delta = cvt_global2local(lidar_odom_point, self.prev_lidar_odom_point)
        self.prev_lidar_odom_point = lidar_odom_point.copy()

        lidar_pf_point = self.pf.localisation(delta, self.scan)
        # rospy.loginfo("cost_function " + str(self.pf.min_cost_function))

        robot_pf_point = find_src(lidar_pf_point, self.lidar_point)
        time3 = time.time()

        self.pub_pf(self.get_odom_frame(robot_pf_point, robot_odom_point))
        time4 = time.time()
        rospy.loginfo("PF RATE: " + str(1 / (time4 - time1)) + " " + str(1 / (time3 - time2)))

    def get_odom(self):
        t = self.buffer.lookup_transform('%s_odom' % self.robot_name, self.robot_name, rospy.Time(0))
        q = [t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w]
        yaw = tf_conversions.transformations.euler_from_quaternion(q)[2]
        return np.array([t.transform.translation.x * 1000, t.transform.translation.y * 1000, yaw])

    def pub_pf(self, point):
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "map"
        t.child_frame_id = "%s_odom" % self.robot_name
        t.transform.translation.x = point[0] / 1000
        t.transform.translation.y = point[1] / 1000
        t.transform.translation.z = 0.0
        q = tf_conversions.transformations.quaternion_from_euler(0, 0, point[2])
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.br.sendTransform(t)


if __name__ == '__main__':
    try:
        rospy.init_node('particle_filter_node', anonymous=True)
        pf_node = PFNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
