#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from EncoderIntegrator import EncoderIntegrator
import numpy as np
from npParticle import ParticleFilter
import matplotlib.pyplot as plt # for DEBUG

# Storage for the latest scan
scan = LaserScan()
coords = np.array([220,383,-np.pi/2]) # set initial coordinates

def odometry_callback(data):
    # parse name,type
    odometry_coords = str(data)[6:].split()
    odometry_coords = np.array(map(float, odometry_coords))

    # calculate coordinates
    lidar_data = np.array([scan.ranges, scan.intensities]).T
    global coords
    coords = particle_filter.localisation(coords, odometry_coords, lidar_data) # TBD: check if input is correct

    # publish calculated coordinates
    pub.publish(' '.join(map(str, coords)))
    
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
    print "odometry_coords:\t", odometry_coords
    print "coords:\t\t", coords
    print "---------"
    
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
        rospy.init_node('particle_filter_node', anonymous=True)
        rospy.Subscriber("scan", LaserScan, scan_callback) # lidar data 
        rospy.Subscriber("odometry_coordinates", String, odometry_callback) # stm data
        pub = rospy.Publisher('coordinates', String, queue_size=1)

        color = "orange"
        particle_filter = ParticleFilter(particles=2000, sense_noise=25, distance_noise=25, angle_noise=0.1, color = color)

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

