#!/usr/bin/env python
import rospy
from std_msgs.msg import String
# import sys
import random


def response_callback(data):
    global is_response
    data_splitted = data.data.split()
    if data_splitted[0] == "move_stm_rand":
        is_response = True


if __name__ == '__main__':
    rospy.init_node("test_move_to_heap")
    pub_move = rospy.Publisher("/main_robot/move_command", String, queue_size=10)
    pub_stm = rospy.Publisher("/main_robot/stm_command", String, queue_size=10)
    rospy.Subscriber("/main_robot/response", String, response_callback)
    rate = rospy.Rate(100)
    rospy.loginfo("Start test move to heap")
    rospy.sleep(1.5)
    for i in range(1):
        x = random.uniform(-0.015, 0.015)
        y = random.uniform(-0.02, 0.02)
        a = random.uniform(-0.15, 0.15)
        is_response = False
        pub_stm.publish("move_stm_rand 162 " + str(x) + " " + str(y) + " " + str(a) + " 0.2 0.2 0.5")
        rospy.loginfo("move to " + str(x) + " " + str(y) + " " + str(a))
        while not is_response and not rospy.is_shutdown():
            rate.sleep()
        rospy.loginfo("movement done")

        is_response = False
        pub_move.publish("move_stm_rand MOVETOHEAP 0")
        rospy.loginfo("move to heap")
        while not is_response and not rospy.is_shutdown():
            rate.sleep()
        rospy.loginfo("move to heap done")
