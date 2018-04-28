#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Header
from sensor_msgs.msg import LaserScan, PointCloud
from geometry_msgs.msg import PoseArray, Pose, Point, Quaternion
import numpy as np
from npParticle import ParticleFilter
import datetime
import tf2_ros
import tf_conversions
from nav_msgs.msg import Odometry

def pf_cmd_callback(data):
    data_splitted = data.data.split()
    cmd = data_splitted[0]
    cmd_args = data_splitted[1:]
    if cmd == 'reset':
        particle_filter.start_over()
    elif cmd == 'clean':
        # clean calibration storage
        particle_filter.clean_storage()
        rospy.loginfo('PF calibration storage cleaned.')
    elif cmd == 'calibrate':
        rospy.loginfo('PF is collecting calibration data.')
        particle_filter.calibrate_beacons(int(cmd_args[0]))
        rospy.loginfo('PF finished collecting calibration data.')
    elif cmd == 'set':
        success, beacons = particle_filter.set_beacons()
        if success:
            rospy.loginfo('PF has calibrated beacon coords:' + str(beacons))
        else:
            rospy.loginfo('Pf failed to calibrate beacon coords.')


def odom_callback(odom):
    quat = odom.pose.pose.orientation
    yaw = tf_conversions.transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])[2]
    global stm_coords
    stm_coords = np.array([odom.pose.pose.position.x * 1000, odom.pose.pose.position.y * 1000, yaw])


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
    # determine time of 1 iteration of PF
    now = datetime.datetime.now()
    global last
    dt = (now - last).total_seconds()
    #print 'PF iteration time in sec:', round(dt, 3), '\t(', round(1 / dt, 1), 'Hz )'
    last = now

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

    #br.sendTransform((pf_coords[0] / 1000, pf_coords[1] / 1000, 0),
    #                                tf.transformations.quaternion_from_euler(0, 0, pf_coords[2]),
    #                                rospy.Time.now(),
    #                                "%s_pf" % robot_name,
    #                                "map")

    # publish tf map -> odom for navigation
    angle = pf_coords[2] - stm_coords[2]
    stm_coords_rotated = rotation_transform(stm_coords, angle)
    br.sendTransform(((pf_coords[0] - stm_coords_rotated[0]) / 1000, (pf_coords[1] - stm_coords_rotated[1]) / 1000, 0),
                tf_conversions.transformations.quaternion_from_euler(0, 0, angle),
                rospy.Time.now(),
                "%s_odom" % robot_name,
                "map")

    # create and pub PointArray with particles    
    #poses = [Pose(Point(x=particle_filter.particles[i, 0] / 1000, y=particle_filter.particles[i, 1] / 1000, z=.4),
    #Quaternion(*quaternion_from_euler(0, 0, particle_filter.particles[i, 2]))) for i in range(len(particle_filter.particles))]
    #particles = PoseArray(header=Header(frame_id="map"), poses=poses)
    #pub_particles.publish(particles)

    # create and pub PointArray with landmarks
    #points = [Point(x=particle_filter.landmarks[0, i] / 1000, y=particle_filter.landmarks[1, i] / 1000, z=.0) for i in range(len(particle_filter.landmarks[0]))]
    #landmarks = PointCloud(header=Header(frame_id="%s_laser" % robot_name), points=points)
    #pub_landmarks.publish(landmarks)

    # DEBUG: pring coordinates of the beacons
    #angle, distance = particle_filter.get_landmarks(lidar_data)
    #x_coords, y_coords = particle_filter.p_trans(angle,distance)
    #np.set_printoptions(formatter={'float': lambda x: "{0:0.2f}".format(x)})
    #print np.array([x_coords, y_coords]).T * 1000
    #print "------------------------------------------"


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
        color = rospy.get_param("/field/color")
        particles = rospy.get_param("particle_filter/particles")
        sense_noise = rospy.get_param("particle_filter/sense_noise")
        distance_noise = rospy.get_param("particle_filter/distance_noise")
        angle_noise = rospy.get_param("particle_filter/angle_noise")

        pf_coords = np.array(rospy.get_param('start_' + color))
        pf_coords[:2] /= 1000.0

        lidar_coords = np.array([rospy.get_param("lidar_x"), rospy.get_param("lidar_y"), rospy.get_param("lidar_a")])
        max_itens = rospy.get_param("particle_filter/max_itens")
        max_dist = rospy.get_param("particle_filter/max_dist")

        in_x, in_y, in_angle = robot2lidar_coords(pf_coords)

        particle_filter = ParticleFilter(particles=particles, sense_noise=sense_noise, distance_noise=distance_noise,
                                         angle_noise=angle_noise, in_x=in_x, in_y=in_y, in_angle=in_angle, color=color,
                                         max_itens=max_itens, max_dist=max_dist)

        # ROS entities
        # Set initial coords and STM (and prev) coords
        coords = np.array([in_x, in_y, in_angle])
        stm_coords = pf_coords.copy()
        prev_used_stm_coords = pf_coords.copy()

        rospy.init_node('particle_filter_node', anonymous=True)
        robot_name = rospy.get_param("robot_name") 
        br = tf2_ros.TransformBroadcaster()

        # for determining time of PF iterations:
        last = datetime.datetime.now()

        rospy.Subscriber("scan", LaserScan, scan_callback, queue_size=1)
        rospy.Subscriber("odom", Odometry, odom_callback, queue_size=1) 
        rospy.Subscriber("pf_cmd", String, pf_cmd_callback, queue_size=1)

        # for vizualization, can be commented before competition
        pub_particles = rospy.Publisher("particle_filter/particles", PoseArray, queue_size=1)
        pub_landmarks = rospy.Publisher("particle_filter/landmarks", PointCloud, queue_size=1)

        ## Simulate
        # start_point = np.array([0.,0.,0.])
        # integrator = EncoderIntegrator(start_point)
        ## usage: 
        ## integrator.integrate(dpoint)
        ## print integrator.last_point

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
