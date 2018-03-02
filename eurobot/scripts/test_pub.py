#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import sys

if __name__ == '__main__':
    rospy.init_node("test_goal_pub")
    pub = rospy.Publisher("/test_pub", String, queue_size=10)
    rospy.sleep(0.2)
    print(sys.argv[1])
    pub.publish(sys.argv[1])
    rospy.sleep(0.5)
