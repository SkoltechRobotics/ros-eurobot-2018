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

# Storage
scan = LaserScan()

def stm_coordinates_callback(data):
    # parse name,type
    stm_coords = data.data.split()
    stm_coords = np.array(map(float, stm_coords))

    # determine time of 1 iteration of PF
    #now = datetime.datetime.now()
    #now_ros = rospy.get_rostime()
    #global last, last_ros
    #print 'python dt =\t', now-last
    #print 'ros dt =\t', now_ros-last_ros
    #last = now
    #last_ros = now_ros

    # calculate coordinates
    lidar_data = np.array([np.array(scan.ranges) * 1000, scan.intensities]).T
    global coords, prev_stm_coords
    coords = particle_filter.localisation(stm_coords - prev_stm_coords, lidar_data)

    # store stm_coords
    prev_stm_coords = stm_coords.copy()

    # publish calculated coordinates
    pub.publish(' '.join(map(str, coords)))
"""
    # create and pub PointArray with particles    
    poses = [Pose(Point(x=particle_filter.particles[i,0]/1000, y=particle_filter.particles[i,1]/1000, z=.4), Quaternion(*quaternion_from_euler(0, 0, particle_filter.particles[i,2]))) for i in range(len(particle_filter.particles))]
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
"""    
def scan_callback(data):
    global scan
    scan = data

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
        color = rospy.get_param("/color")
        particles = rospy.get_param("/main_robot/particles")
        sense_noise = rospy.get_param("/main_robot/sense_noise")
        distance_noise = rospy.get_param("/main_robot/distance_noise")
        angle_noise = rospy.get_param("/main_robot/angle_noise")
        in_x = rospy.get_param("/main_robot/start_x")
        in_y = rospy.get_param("/main_robot/start_y")
        in_angle = rospy.get_param("/main_robot/start_a")
        max_itens = rospy.get_param("/main_robot/max_itens")
        max_dist = rospy.get_param("/main_robot/max_dist")
        particle_filter = ParticleFilter(particles=particles, sense_noise=sense_noise, distance_noise=distance_noise, angle_noise=angle_noise, in_x=in_x, in_y=in_y, in_angle=in_angle, color = color, max_itens=max_itens, max_dist=max_dist)

        # ROS entities
        # Set initial coords and previous STM coords
        global coords, prev_stm_coords
        coords = np.array([rospy.get_param('/main_robot/start_x'), rospy.get_param('/main_robot/start_y'), rospy.get_param('/main_robot/start_a')])
        prev_stm_coords = coords.copy()

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

