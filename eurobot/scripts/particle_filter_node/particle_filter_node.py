#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Header
from sensor_msgs.msg import LaserScan, PointCloud
from geometry_msgs.msg import PoseArray, Pose, Point, Quaternion
import numpy as np
from npParticle import ParticleFilter, cvt_global2local, cvt_local2global
import datetime
import tf
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler

PARTICLES = 500
SENSE_NOISE = 50
DISTANCE_NOISE = 5
ANGLE_NOISE = 0.5

class PFNode(object):
    def __init__(self):
        # Init params
        self.color = rospy.get_param("/field/color")
        self.robot_name = rospy.get_param("robot_name")
        lidar_pf_point = np.array(rospy.get_param('start_' + self.color))
        lidar_pf_point[:2] /= 1000.0

        self.scan = None
        self.listener = tf.TransformListener()

    def scan_callback(self, scan):
        self.scan = np.array([np.array(scan.ranges) * 1000, scan.intensities]).T

    def localisation(self):
        odom_robot_point = self.get_odom()
        delta = cvt_global2local()

    def get_odom(self):
        (trans, rot) = self.listener.lookupTransform('/%s_odom' % self.robot_name, self.robot_name, rospy.Time(0))
        yaw = tf.transformations.euler_from_quaternion(rot)[2]
        return np.array([trans[0], trans[1], yaw])

def delta_coords(used_stm_coords, prev_used_stm_coords, pf_coords):
    """ Calculates delta coords for PF, that is used for moving particles """
    delta = used_stm_coords - prev_used_stm_coords
    # Actual delta is different, as stm_coords (from odometry) accumulates an error. fix:
    return rotation_transform(delta, pf_coords[2] - prev_used_stm_coords[2])


def rotation_transform(vec, angle):
    """ Rotation transformation of 2D vector given in form of 1D array"""
    M = np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])
    ans = vec.copy()
    ans[:2] = np.matmul(M, ans[:2].reshape((2,1))).reshape(2)
    return ans 


def scan_callback(scan):
    global coords, stm_coords, prev_used_stm_coords, pf_coords
    # copy stm_coords as they may change during calculations
    used_stm_coords = stm_coords.copy()

    # calculate coordinates
    # TBD: check if python lib for Hokuyo is faster than urg_node and this transformation
    lidar_data = np.array([np.array(scan.ranges) * 1000, scan.intensities]).T
    delta = delta_coords(used_stm_coords, prev_used_stm_coords, pf_coords)
    
    coords = particle_filter.localisation(delta, lidar_data)

    #print 'PF work time in sec:', datetime.datetime.now() - last
    # store previous stm_coords
    prev_used_stm_coords = used_stm_coords.copy()

    # robot may have moved a bit during calculations. fix:
    coords += delta_coords(stm_coords, used_stm_coords, coords)
    # rate of updating stm_coords should be higher that scanning rate

    global lidar_coords, in_x, in_y, in_angle, br, robot_name
    pf_coords = lidar2robot_coords(coords)

    angle = pf_coords[2] - stm_coords[2]
    stm_coords_rotated = rotation_transform(stm_coords, angle)
    br.sendTransform(((pf_coords[0] - stm_coords_rotated[0]) / 1000, (pf_coords[1] - stm_coords_rotated[1]) / 1000, 0),
                tf.transformations.quaternion_from_euler(0, 0, angle),
                rospy.Time.now(),
                "%s_odom" % robot_name,
                "map")


def robot2lidar_coords(coords_of_robot):
    global lidar_coords
    ans = coords_of_robot + rotation_transform(lidar_coords, coords_of_robot[2])
    ans[2] = ans[2] % (2 * np.pi)
    return ans

def lidar2robot_coords(coords_of_lidar):
    global lidar_coords
    ans = coords_of_lidar - rotation_transform(lidar_coords, coords_of_lidar[2] - lidar_coords[2])
    ans[2] = ans[2] % (2 * np.pi)
    return ans

if __name__ == '__main__':
    try:
        # create a PF object with params from ROS


        pf_coords = np.array(rospy.get_param('start_' + color))
        pf_coords[:2] /= 1000.0

        lidar_coords = np.array([rospy.get_param("lidar_x"), rospy.get_param("lidar_y"), rospy.get_param("lidar_a")])
        max_itens = rospy.get_param("particle_filter/max_itens")
        max_dist = rospy.get_param("particle_filter/max_dist")

        in_x, in_y, in_angle = robot2lidar_coords(pf_coords)

        particle_filter = ParticleFilter(particles_num=particles, sense_noise=sense_noise, distance_noise=distance_noise,
                                         angle_noise=angle_noise, start_x=in_x, start_y=in_y, start_angle=in_angle, color=color,
                                         max_itens=max_itens, max_dist=max_dist)

        # ROS entities
        # Set initial coords and STM (and prev) coords
        coords = np.array([in_x, in_y, in_angle])
        stm_coords = pf_coords.copy()
        prev_used_stm_coords = pf_coords.copy()

        rospy.init_node('particle_filter_node', anonymous=True)
        robot_name = rospy.get_param("robot_name") 
        br = tf.TransformBroadcaster()

        # for determining time of PF iterations:
        last = datetime.datetime.now()

        rospy.Subscriber("scan", LaserScan, scan_callback, queue_size=1)
        rospy.Subscriber("odom", Odometry, odom_callback, queue_size=1) 
        rospy.Subscriber("pf_cmd", String, pf_cmd_callback, queue_size=1)

        # for vizualization, can be commented before competition
        pub_particles = rospy.Publisher("particle_filter/particles", PoseArray, queue_size=1)
        pub_landmarks = rospy.Publisher("particle_filter/landmarks", PointCloud, queue_size=1)

        rospy.spin()
    except rospy.ROSInterruptException:
        pass
