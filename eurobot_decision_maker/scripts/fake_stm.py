#! /usr/bin/env python

import rospy
from std_msgs.msg import String

import re

pub = None

def fake0(msg):
    rospy.loginfo(msg.data)

def fake_responser(time):
    def fake_response(msg):
        r = rospy.Rate(time)
        rospy.loginfo(msg.data)
        action_name = re.match(r"(\S*)\s[\s\S]*",msg.data).group(1)
        r.sleep()
        response_message = action_name + ' ' + 'done'
        global pub
        pub.publish(response_message)
        rospy.loginfo("wrote " + response_message)
    return fake_response    

if __name__  == '__main__':
    rospy.init_node('fake_stm', anonymous =True)
    
    pub = rospy.Publisher('/main_robot/response', String, queue_size = 10)
    sub = rospy.Subscriber('/main_robot/stm_command', String, fake_response(5))
    rospy.loginfo('Started')
    
    rospy.spin()
