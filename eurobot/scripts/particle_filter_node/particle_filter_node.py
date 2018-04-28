#!/usr/bin/env python
import rospy
from geometry_msgs.msg import TransformStamped
import numpy as np
from npParticle import ParticleFilter, cvt_global2local, cvt_local2global, find_src
import tf2_ros
import tf_conversions

PF_RATE = 10

PARTICLES = 500
SENSE_NOISE = 10
DISTANCE_NOISE = 1
ANGLE_NOISE = 0.15

MIN_INTENS = 3500
MAX_DIST = 3000
BEAC_DIST_THRES = 200


class PFNode(object):
    # noinspection PyTypeChecker
    def __init__(self):
        # Init params
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
        self.pf = ParticleFilter(PARTICLES, SENSE_NOISE, DISTANCE_NOISE, ANGLE_NOISE, x, y, a, self.color)

        rospy.Timer(rospy.Duration(1. / PF_RATE), self.localisation)

    def scan_callback(self, scan):
        self.scan = np.array([np.array(scan.ranges) * 1000, scan.intensities]).T

    def localisation(self):
        robot_odom_point = self.get_odom()
        lidar_odom_point = cvt_local2global(self.lidar_point, robot_odom_point)
        delta = cvt_global2local(lidar_odom_point, self.prev_lidar_odom_point)
        self.prev_lidar_odom_point = lidar_odom_point.copy()
        lidar_pf_point = self.pf.localisation(delta, self.scan)
        robot_pf_point = find_src(lidar_pf_point, self.lidar_point)
        self.pub_pf(find_src(robot_pf_point, robot_odom_point))

    def get_odom(self):
        (trans, rot) = self.buffer.lookup_transform('/%s_odom' % self.robot_name, self.robot_name, rospy.Time(0))
        yaw = tf_conversions.transformations.euler_from_quaternion(rot)[2]
        return np.array([trans[0], trans[1], yaw])

    def pub_pf(self, point):
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "map"
        t.child_frame_id = "%s_odom" % self.robot_name
        t.transform.translation.x = point[0]
        t.transform.translation.y = point[1]
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
