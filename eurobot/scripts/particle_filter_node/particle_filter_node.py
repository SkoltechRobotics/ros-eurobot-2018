#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Header
from sensor_msgs.msg import LaserScan, PointCloud
from geometry_msgs.msg import PoseArray, Pose, Point, Quaternion
from EncoderIntegrator import EncoderIntegrator
import numpy as np
from npParticle import ParticleFilter
from tf.transformations import quaternion_from_euler
import datetime

def stm_coordinates_callback(data):
    global stm_coords
    stm_coords = np.array(map(float, data.data.split()))

def delta_coords(used_stm_coords, prev_used_stm_coords, pf_coords):
    """ Calculates delta coords for PF, that is used for moving particles """
    delta = used_stm_coords - prev_used_stm_coords
    # Actual delta is different, as stm_coords (from odometry) accumulates an error. fix:
    stm_error_angle = pf_coords[2] - prev_used_stm_coords[2]
    delta[0] *= np.cos(stm_error_angle)
    delta[1] *= np.sin(stm_error_angle)
    return delta
    
def scan_callback(scan):
    # determine time of 1 iteration of PF
    #now = datetime.datetime.now()
    #now_ros = rospy.get_rostime()
    #global last, last_ros
    #print 'python dt =\t', now-last
    #print 'ros dt =\t', now_ros-last_ros
    #last = now
    #last_ros = now_ros

    global coords, stm_coords, prev_used_stm_coords
    # copy stm_coords as they may change during calculations
    used_stm_coords = stm_coords.copy()

    # calculate coordinates
    # TBD: check if python lib for Hokuyo is fasteer than urg_node and this transformation
    lidar_data = np.array([np.array(scan.ranges) * 1000, scan.intensities]).T
    delta = delta_coords(used_stm_coords, prev_used_stm_coords, coords)
    coords = particle_filter.localisation(delta, lidar_data)

    # store previous stm_coords
    prev_used_stm_coords = used_stm_coords.copy()

    # robot may have moved a bit during calculations. fix:
    coords += delta_coords(stm_coords,used_stm_coords, coords)
    # rate of updating stm_coords should be higher that scanning rate

    # publish calculated coordinates
    pub.publish(' '.join(map(str, coords)))

    # create and pub PointArray with particles    
    poses = [Pose(Point(x=particle_filter.particles[i,0]/1000, y=particle_filter.particles[i,1]/1000, z=.4), Quaternion(*quaternion_from_euler(0, 0, particle_filter.particles[i,2]+np.pi/2))) for i in range(len(particle_filter.particles))]
    header = Header(frame_id="world")
    particles = PoseArray(header=header, poses=poses)
    pub_particles.publish(particles)
    
    # create and pub PointArray with landmarks
    points = [Point(x=particle_filter.landmarks[0,i], y=particle_filter.landmarks[1,i], z=.0) for i in range(len(particle_filter.landmarks[0]))]
    header = Header(frame_id="laser")
    landmarks = PointCloud(header=header, points=points)
    pub_landmarks.publish(landmarks)
    
    # DEBUG
    #print "Landmarks:"
    #landm = particle_filter.get_landmarks(lidar_data)
    #print particle_filter.p_trans(landm[0],landm[1])
    #print "---------"
    
    ## DEBUG: pring coordinates of the beacons
    #lidar_data = np.array([scan.ranges, scan.intensities]).T
#angle, distance = particle_filter.get_landmarks(lidar_data)
#x_coords, y_coords = particle_filter.p_trans(angle,distance)
    #np.set_printoptions(formatter={'float': lambda x: "{0:0.2f}".format(x)})
    #print np.array([x_coords, y_coords]).T * 1000
    #print "------------------------------------------"

if __name__ == '__main__':
    try:
        # create a PF object with params from ROS
        color = rospy.get_param("/field/color")
        particles = rospy.get_param("particle_filter/particles")
        sense_noise = rospy.get_param("particle_filter/sense_noise")
        distance_noise = rospy.get_param("particle_filter/distance_noise")
        angle_noise = rospy.get_param("particle_filter/angle_noise")
        in_x = rospy.get_param("start_x")
        in_y = rospy.get_param("start_y")
        in_angle = rospy.get_param("start_a")
        max_itens = rospy.get_param("particle_filter/max_itens")
        max_dist = rospy.get_param("particle_filter/max_dist")
        particle_filter = ParticleFilter(particles=particles, sense_noise=sense_noise, distance_noise=distance_noise, angle_noise=angle_noise, in_x=in_x, in_y=in_y, in_angle=in_angle, color = color, max_itens=max_itens, max_dist=max_dist)

        # ROS entities
        # Set initial coords and STM (and prev) coords
        coords = np.array([in_x, in_y, in_angle])
        stm_coords = coords.copy()
        prev_used_stm_coords = coords.copy()

        rospy.init_node('particle_filter_node', anonymous=True)

        # for determining time of PF iterations:
        #last = datetime.datetime.now()
        #last_ros = rospy.get_rostime()

        rospy.Subscriber("scan", LaserScan, scan_callback, queue_size=1) # lidar data 
        rospy.Subscriber("stm/coordinates", String, stm_coordinates_callback, queue_size=1) # stm data
        pub = rospy.Publisher('particle_filter/coordinates', String, queue_size=1)
        # for vizualization, can be commented before competition
        pub_particles = rospy.Publisher("particle_filter/particles", PoseArray, queue_size=1)
        pub_landmarks = rospy.Publisher("particle_filter/filtered_scan", PointCloud, queue_size=1)


        ## Simulate
        #start_point = np.array([0.,0.,0.])
        #integrator = EncoderIntegrator(start_point)
        ## usage: 
        ## integrator.integrate(dpoint)
        ## print integrator.last_point

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

