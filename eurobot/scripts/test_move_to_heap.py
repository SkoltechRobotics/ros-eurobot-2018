#!/usr/bin/env python
import rospy
from std_msgs.msg import String
# import sys
import random


def response_callback(msg):
    global is_response
    is_response = True


if __name__ == '__main__':
    rospy.init_node("test_move_to_heap")
    pub_move = rospy.Publisher("/main_robot/move_command", String, queue_size=10)
    pub_stm = rospy.Publisher("/main_robot/stm_command", String, queue_size=10)
    rospy.Subscriber("/main_robot/response", String, response_callback)
    rate = rospy.Rate(100)
    for i in range(10):
        x = random.uniform(-0.03, 0.03)
        y = random.uniform(-0.03, 0.03)
        a = random.uniform(-0.3, 0.3)
        is_response = False
        pub_stm.publish("move_stm_rand 162 " + str(x) + " " + str(y) + " " + str(a) + " 0.2 0.2 0.5")
        while not is_response:
            rate.sleep()

        is_response = False
        pub_move.publish("move_to_heap_rand MOVETOHEAP 0")
        while not is_response:
            rate.sleep()
