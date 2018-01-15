#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from EncoderIntegrator import EncoderIntegrator
import numpy as np
from npParticle import ParticleFilter
#import matplotlib.pyplot as plt # for DEBUG

# Storage for the latest scan
scan = LaserScan()
coords = np.array([220,383,-np.pi/2]) # set initial coordinates

def stm_coordinates_callback(data): # TBD: recieve coords, not delta_coords
    # parse name,type
    delta_coords = str(data)[6:].split()
    delta_coords = np.array(map(float, delta_coords))

    # transform into field coordinate system
    tmp = []
    tmp.append(delta_coords[0]*np.cos(coords[2]) - delta_coords[1]*np.sin(coords[2]))
    tmp.append(delta_coords[1]*np.cos(coords[2]) + delta_coords[0]*np.sin(coords[2]))
    tmp.append(delta_coords[2])
    delta_coords = tmp

    # calculate coordinates
    lidar_data = np.array([scan.ranges, scan.intensities]).T
    global coords
    coords = particle_filter.localisation(delta_coords, lidar_data)

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
    print "delta_coords:\t", delta_coords
    print "coords:\t\t", coords
    #print "Landmarks:"
    #landm = particle_filter.get_landmarks(lidar_data)
    #print particle_filter.p_trans(landm[0],landm[1])
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
        rospy.Subscriber("stm/coordinates", String, stm_coordinates_callback) # stm data
        pub = rospy.Publisher('particle_filter/coordinates', String, queue_size=1)

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

