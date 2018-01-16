#! /usr/bin/env python

import rospy
from std_msgs.msg import String

import re

pub = None

def fake_response(msg):
    rospy.loginfo(msg.data)
    action_id = re.match('(\S*)\s([\S\s]*)',msg.data).group(1)
    global pub
    pub.publish(action_id + ' running')
    rospy.sleep(0.1)
    pub.publish(action_id + ' finished')

if __name__ == '__main__':
    rospy.init_node('fake_stm', anonymous=True)
    
    pub = rospy.Publisher('fake_stm_response', String, queue_size=10)
    sub = rospy.Subscriber('fake_stm_command', String, fake_response)

    rospy.spin()

