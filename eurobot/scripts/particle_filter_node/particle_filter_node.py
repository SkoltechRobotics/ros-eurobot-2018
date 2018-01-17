#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Header
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseArray, Pose, Point, Quaternion
from EncoderIntegrator import EncoderIntegrator
import numpy as np
from npParticle import ParticleFilter
#import matplotlib.pyplot as plt # for DEBUG

# Storage
scan = LaserScan()

def stm_coordinates_callback(data):
    # parse name,type
    stm_coords = data.data.split()
    stm_coords = np.array(map(float, stm_coords))

    # calculate coordinates
    lidar_data = np.array([scan.ranges, scan.intensities]).T
    global coords, prev_stm_coords
    coords = particle_filter.localisation(stm_coords - prev_stm_coords, lidar_data)

    # store stm_coords
    prev_stm_coords = stm_coords

    # publish calculated coordinates
    pub.publish(' '.join(map(str, coords)))

    poses = [Pose(position=Point(x=particle_filter.particles[i,0], y=particle_filter.particles[i,1]), orientation=Quaternion(w=particle_filter.particles[i,2])) for i in range(len(particle_filter.particles))]
    particles = PoseArray(header=Header(frame_id="particles"), poses=PoseArray(header=Header(frame_id="main_robot_stm"), poses=poses))
    pub_particles.publish(particles)
    #filtered_scan = PointCloud2(header=Header(frame_id="laser"))
    #pub_filtered.publish(filtered_scan)
    
    # DEBUG
    # visualise landmarks
    #angle, distance = particle_filter.get_landmarks(lidar_data)
    #x_coords, y_coords = particle_filter.p_trans(angle,distance)
    #plt.plot(x_coords, y_coords, 'r.')
    #plt.xlim(-3, 3)
    #plt.ylim(-3, 3)
    #plt.gca().set_aspect('equal', adjustable='box')
    #plt.show()
    
    # DEBUG
    #print "Landmarks:"
    #landm = particle_filter.get_landmarks(lidar_data)
    #print particle_filter.p_trans(landm[0],landm[1])
    #print "---------"
    
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
        # ROS entities
        # Set initial coords and previous STM coords
        global coords, prev_stm_coords
        coords = np.array([rospy.get_param('/main_robot/start_x'), rospy.get_param('/main_robot/start_y'), rospy.get_param('/main_robot/start_a')])
        prev_stm_coords = coords.copy()

        rospy.init_node('particle_filter_node', anonymous=True)
        rospy.Subscriber("scan", LaserScan, scan_callback) # lidar data 
        rospy.Subscriber("stm/coordinates", String, stm_coordinates_callback) # stm data
        pub = rospy.Publisher('particle_filter/coordinates', String, queue_size=1)
        # for vizualization, can be commented before competition
        pub_particles = rospy.Publisher("particle_filter/particles", PoseArray, queue_size=1)
        pub_filtered = rospy.Publisher("particle_filter/filtered_scan", PoseArray, queue_size=1)

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

