#! /usr/bin/env python

import rospy
from std_msgs.msg import String

import re
import sys

pub = None

def fake_response(msg):
    rospy.loginfo(msg.data)
    action_id = re.match('(\S*)\s([\S\s]*)',msg.data).group(1)
    global pub
    pub.publish(action_id + ' running')
    
    def timed_callback(e):
        pub.publish(action_id + ' finished')

    t = rospy.Timer(rospy.Duration(0.5), timed_callback)

    #pub.publish(action_id + ' finished')
    

if __name__ == '__main__':
    rospy.init_node('fake_stm', anonymous=True)
    topic_names = ['fake_stm_response', 'fake_stm_command']
    if len(sys.argv) > 1:
        topic_names = sys.argv[1:]
    pub = rospy.Publisher(topic_names[0], String, queue_size=10)
    subs = [rospy.Subscriber(sub_name, String, fake_response) for sub_name in topic_names[1:]]

    rospy.spin()

