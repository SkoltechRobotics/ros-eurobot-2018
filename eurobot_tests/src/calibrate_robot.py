#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import sys
import time
import random
import tf
import numpy as np


def response_callback(data):
    global is_response
    data_splitted = data.data.split()
    if data_splitted[0] == movement_id:
        is_response = True


def start_cmd(cmd):
    global is_response
    rospy.loginfo("Start cmd: " + cmd)
    is_response = False
    pub_stm.publish(cmd)
    start_time = time.time()
    while not is_response and not rospy.is_shutdown():
        rate.sleep()
    rospy.loginfo("End cmd with time " + str(time.time() - start_time))


def take_angle():
    (trans, rot) = listener.lookupTransform('map', 'secondary_robot', rospy.Time(0))
    yaw = tf.transformations.euler_from_quaternion(rot)[2]
    return yaw


def take_x():
    (trans, rot) = listener.lookupTransform('map', 'secondary_robot', rospy.Time(0))
    return trans[0]


if __name__ == '__main__':
    rospy.init_node("test_stm_secondary_main_robot")
    pub_stm = rospy.Publisher("/secondary_robot/stm_command", String, queue_size=10)
    rospy.Subscriber("/secondary_robot/response", String, response_callback)
    rate = rospy.Rate(100)
    rospy.loginfo("Start test move stm")
    rospy.sleep(1.2)
    listener = tf.TransformListener()
    rospy.sleep(0.5)

    is_response = False
    movement_id = "test_stm_main_robot"
    n = int(sys.argv[2])
    if sys.argv[1] == "rotation":
        v = float(sys.argv[3])
        last_angle = take_angle()
        errors = []
        for i in range(n):
            rospy.loginfo("----------------------------------")
            rospy.sleep(1)
            start_cmd(movement_id + ' 162 0 0 3.1415 0 0 ' + str(v))
            angle = take_angle()
            diff_angle = (angle - last_angle) % (2 * np.pi)
            last_angle = angle
            error = diff_angle - np.pi
            errors.append(error)
            rospy.loginfo("moved angle " + str(diff_angle))
            rospy.loginfo("error " + str(error))

        errors = np.array(errors)
        rospy.loginfo("MEAN ERROR " + str(np.mean(errors)) + " " + str(np.mean(errors) / np.pi))
        rospy.loginfo("STD ERROR " + str(np.std(errors)))
    elif sys.argv[1] == "linear_x":
        v = float(sys.argv[3])
        last_x = take_x()
        errors = []
        for i in range(n):
            rospy.loginfo("----------------------------------")
            rospy.sleep(1)
            if i % 2:
                start_cmd(movement_id + ' 162 0.2 0 0 ' + str(v) + ' 0 0')
            else:
                start_cmd(movement_id + ' 162 -0.2 0 0 ' + str(v) + ' 0 0')

            x = take_x()
            diff_x = x - last_x
            last_x = x
            if i % 2:
                error = diff_x - 0.2
            else:
                error = diff_x + 0.2

            errors.append(error)
            rospy.loginfo("moved x " + str(diff_x))
            rospy.loginfo("error " + str(error))
        errors = np.array(errors)
        rospy.loginfo("MEAN ERROR " + str(np.mean(errors)) + " " + str(np.mean(errors) / 0.2))
        rospy.loginfo("STD ERROR " + str(np.std(errors)))