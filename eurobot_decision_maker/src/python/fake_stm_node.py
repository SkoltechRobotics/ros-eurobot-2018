#! /usr/bin/env python

import rospy
from std_msgs.msg import String

import re

pub = None

def fake0(msg):
    rospy.loginfo(msg.data)

def fake_response(msg):
    r = rospy.Rate(1)
    rospy.loginfo(msg.data)
    action_name = re.match(r"(\S*)\s[\s\S]*",msg.data).group(1)
    r.sleep()
    response_message = action_name + ' ' + 'done'
    global pub
    pub.publish(response_message)
    rospy.loginfo("wrote " + response_message)
    

if __name__  == '__main__':
    rospy.init_node('fake_stm', anonymous =True)
    
    pub = rospy.Publisher('stm_response', String, queue_size = 10)
    sub = rospy.Subscriber('stm_command', String, fake_response)
    rospy.loginfo('Started')
    
    rospy.spin()
