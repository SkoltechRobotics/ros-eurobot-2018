#! /usr/bin/env python

import rospy
from std_msgs.msg import String

import re

pub = None


def fake0(msg):
    rospy.loginfo(msg.data)


def fake_responser(time):
    def fake_response(msg):
        rospy.loginfo(msg.data)
        action_name = re.match(r"(\S*)\s[\s\S]*", msg.data).group(1)
        response_message = action_name + ' ' + 'finished'
        global pub
        def cb(e):
            pub.publish(response_message)
            rospy.loginfo("wrote " + response_message)
        rospy.Timer(rospy.Duration(time), cb, oneshot=True)

    return fake_response


if __name__ == '__main__':
    rospy.init_node('fake_stm', anonymous=True)

    pub = rospy.Publisher('/main_robot/response', String, queue_size=10)
    sub = rospy.Subscriber('/main_robot/stm_command', String, fake_responser(4))
    rospy.loginfo('Started')

    rospy.spin()
