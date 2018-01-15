#! /usr/bin/env python

import rospy
from std_msgs.msg import String

import re

pub = None


if __name__  == '__main__':
    pub = rospy.Publisher('commander', String, queue_size = 100)
    rospy.init_node('actions_launcher', anonymous =True)
    

    #sub = rospy.Subscriber('robot_command', String, fake_response)
    data = ""
    with open("tree_description.txt") as btf:
        for s in btf.readlines()[1:]:
            aname, atype, aparent = s.split(' ')
            #print(atype)
            if atype == 'action':
                pub.publish(aname + ' ' + atype + ' '+ '0 0 0')
                rospy.loginfo(aname + ' ' + atype + ' '+ '0 0 0')
    rospy.spin()
