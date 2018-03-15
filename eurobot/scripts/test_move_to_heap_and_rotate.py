#!/usr/bin/env python
import rospy
from std_msgs.msg import String
# import sys
import random
import numpy as np


def add_heap_rotation(self, parent_name, angle):
    radius = 0.06  # m
    linear_speed = 0.05  # m/s
    self.last_coordinates[-1] += angle
    self.last_coordinates[-1] %= 2 * np.pi
    angle = angle % (2 * np.pi)
    angle = (angle + np.pi) % (2 * np.pi) - np.pi
    angle = np.fix(angle * 1000) / 1000
    self.add_command_action(parent_name, 162, radius * angle, 0, angle, linear_speed, 0, linear_speed / radius)


def response_callback(msg):
    global is_response
    is_response = True


if __name__ == '__main__':
    rospy.init_node("test_move_to_heap")
    pub_move = rospy.Publisher("/main_robot/move_command", String, queue_size=10)
    pub_stm = rospy.Publisher("/main_robot/stm_command", String, queue_size=10)
    rospy.Subscriber("/main_robot/response", String, response_callback)
    rate = rospy.Rate(100)
    rospy.loginfo("Start test move to heap")
    rospy.sleep(1.5)
    for i in range(4):
        radius = 0.06
        rotation_speed = 1
        angle = -np.pi / 2
        x = radius * angle
        y = 0
        a = angle
        speed_x = radius * rotation_speed
        speed_y = 0
        speed_a = rotation_speed
        is_response = False
        pub_stm.publish("move_stm_rand 162 " + ' '.join(map(str, [x, y, a, speed_x, speed_y, speed_a])))
        rospy.loginfo("move to " + str(x) + " " + str(y) + " " + str(a))
        while not is_response and not rospy.is_shutdown():
            rate.sleep()
        rospy.loginfo("movement done")

        is_response = False
        pub_move.publish("move_to_heap_rand MOVETOHEAP 0")
        rospy.loginfo("move to heap")
        while not is_response and not rospy.is_shutdown():
            rate.sleep()
        rospy.loginfo("move to heap done")
