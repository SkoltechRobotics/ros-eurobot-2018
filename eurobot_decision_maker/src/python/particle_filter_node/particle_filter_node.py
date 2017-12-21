#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from EncoderIntegrator import EncoderIntegrator
import numpy as np


def odometry_callback(data):
    # parse name,type
    dpoint = str(data)[6:].split()
    dpoint = np.array([float(dpoint[i]) for i in range(3)])

    # calculate coordinates
    # for now it's just odometry, actual filter + lidar are TBD
    integrator.integrate(dpoint)

    # publish calculated coordinates
    pub_response.publish(str(integrator.last_point[0])+' '+str(integrator.last_point[1])+' '+str(integrator.last_point[2]))
    
    rate.sleep()

if __name__ == '__main__':
    try:
        start_point = np.array([0.,0.,0.])
        integrator = EncoderIntegrator(start_point)
        rospy.init_node('particle_filter_node', anonymous=True)
        rate = rospy.Rate(100)
        pub_response = rospy.Publisher("particle_filter_coordinates", String, queue_size=10)
        rospy.Subscriber("odometry_coordinates", String, odometry_callback)
        #rospy.Subscriber("slice", String, slice_callback) # lidar data 

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

