#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from EncoderIntegrator import EncoderIntegrator
import numpy as np
from npParticle import ParticleFilter

# Storage for the latest scan
scan = LaserScan()
coords = np.array([0.383, 0.220,-np.pi/2]) # set initial coordinates

def odometry_callback(data):
    # parse name,type
    odometry_coords = str(data)[6:].split()
    odometry_coords = np.array(map(float, odometry_coords))

    # calculate coordinates
    lidar_data = np.array([scan.ranges, scan.intensities]).T
    global coords
    coords = particle_filter.localisation(coords, odometry_coords, lidar_data) # TBD: check if input is correct
    coords[0] = coords[0]/1000
    coords[1] = coords[1]/1000 # TBD: check division by 1000

    # publish calculated coordinates
    pub.publish(' '.join(map(str, coords)))
    
    # DEBUG
    #print odometry_coords
    
def scan_callback(data):
    global scan
    scan = data
    
    ## DEBUG: pring coordinates of the beacons
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

