#! /usr/bin/env python

import rospy
from std_msgs.msg import String

import re

if __name__  == '__main__':
    pub = rospy.Publisher('bt_maker', String, queue_size = 10)
    
    rospy.init_node('tree_description', anonymous =True)
    #sub = rospy.Subscriber('robot_command', String, fake_response)
    data = ""
    with open("tree_description.txt") as btf:
        data = btf.read().replace('\n',' ')
    rospy.loginfo(data)
    
    pub.publish(data)
    rospy.spin()
