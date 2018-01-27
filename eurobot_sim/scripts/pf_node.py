#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import datetime
import numpy as np


def stm_coord_callback(data):
    global pub
    data_splitted = data.data.split()
    point = np.array([float(x) for x in data_splitted[:3]])
    point += np.random.normal(scale=(10, 10, 0.02), size=3)
    pub.publish(' '.join(map(str, point)))
 

if __name__ == '__main__':
    try:
        rospy.init_node('pf_sim_node', anonymous=True)
        pub = rospy.Publisher("particle_filter/coordinates", String, queue_size=10)
        rospy.Subscriber("stm/coordinates", String, stm_coord_callback)

        rospy.spin()
    except rospy.ROSInterruptException:
        pass
