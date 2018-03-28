#!/usr/bin/env python
import rospy
from std_msgs.msg import String
# import sys
import random
import numpy as np
has_moved = False

def add_heap_rotation(self, parent_name, angle):
    radius = 0.06  # m
    linear_speed = 0.05  # m/s
    self.last_coordinates[-1] += angle
    self.last_coordinates[-1] %= 2 * np.pi
    angle = angle % (2 * np.pi)
    angle = (angle + np.pi) % (2 * np.pi) - np.pi
    angle = np.fix(angle * 1000) / 1000
    self.add_command_action(parent_name, 162, radius * angle, 0, angle, linear_speed, 0, linear_speed / radius)

def wait_for_movement(name):
    global has_moved
    has_moved = False
    def cb(msg):
        global has_moved
        if msg.data == name + " finished":
            has_moved = True
    rospy.Subscriber("/main_robot/response", String, cb)
    while not has_moved:
        rospy.sleep(0.1)
                                                                                            
if __name__ == '__main__':
    rospy.init_node("test_move_to_heap")
    pub_move = rospy.Publisher("/main_robot/move_command", String, queue_size=10)
    pub_stm = rospy.Publisher("/main_robot/stm_command", String, queue_size=10)
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
        pub_stm.publish("move_stm_rand 162 " + ' '.join(map(str, [x, y, a, speed_x, speed_y, speed_a])))
        rospy.loginfo("move to " + str(x) + " " + str(y) + " " + str(a))
        wait_for_movement("move_stm_rand")
        rospy.loginfo("movement done")
        rospy.sleep(0.5)

        pub_move.publish("move_to_heap_rand MOVETOHEAP 0")
        rospy.loginfo("move to heap")
        wait_for_movement("move_to_heap_rand")
        rospy.loginfo("move to heap done")
