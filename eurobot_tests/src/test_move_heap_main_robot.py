#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import sys
import time
import random
import numpy as np

PI = 3.1415926
PARAMS_PI2P = [1.019 * PI / 2, 0.096, 0.014]
PARAMS_PI2M = [-1.02 * PI / 2, -0.096, 0.01]
# (number of repeats) rotation (rotation velocity)
CONFIGURATIONS = {(1, 1, 1): 0,
                  (0, 1, 1): 3,
                  (1, 0, 1): 2,
                  (1, 1, 0): 1,
                  (1, 0, 0): 4,
                  (0, 1, 0): 5,
                  (0, 0, 1): 6}


def response_callback(data):
    global is_response
    data_splitted = data.data.split()
    if data_splitted[0] == movement_id:
        is_response = True


def start_stm_cmd(cmd):
    global is_response
    rospy.loginfo("Start stm cmd: " + cmd)
    is_response = False
    pub_stm.publish(cmd)
    start_time = time.time()
    while not is_response and not rospy.is_shutdown():
        rate.sleep()
    rospy.loginfo("End cmd with time " + str(time.time() - start_time))


def start_move_cmd(cmd):
    global is_response
    rospy.loginfo("Start move cmd: " + cmd)
    is_response = False
    pub_move.publish(cmd)
    start_time = time.time()
    while not is_response and not rospy.is_shutdown():
        rate.sleep()
    rospy.loginfo("End cmd with time " + str(time.time() - start_time))


if __name__ == '__main__':
    rospy.init_node("test_stm_move_main_robot")
    pub_stm = rospy.Publisher("/main_robot/stm_command", String, queue_size=10)
    pub_move = rospy.Publisher("/main_robot/move_command", String, queue_size=10)
    rospy.Subscriber("/main_robot/response", String, response_callback)
    rate = rospy.Rate(100)
    rospy.loginfo("Start test move stm move")
    rospy.sleep(1.5)

    is_response = False
    movement_id = "test_stm_main_robot"
    n = int(sys.argv[1])

    p = 0
    for i in range(n):
        if rospy.is_shutdown():
            break
        if sys.argv[2] == "rotation":
            rospy.sleep(0.5)
            w = 3
            k = random.randint(0, 1)
            if k:
                angle, x, y = PARAMS_PI2P
            else:
                angle, x, y = PARAMS_PI2M
            rospy.loginfo("Start rotation on angle " + str(angle) + " with velocity " + str(w))
            start_stm_cmd(movement_id + ' 162 ' + ' '.join(map(str, [x, y, angle, abs(x / angle) * w, abs(y / angle) * w, w])))
            rospy.loginfo("Start move to heap configuration 0")
            start_move_cmd(movement_id + " MOVETOHEAP 0")
        if sys.argv[2] == "bad_rotation":
            rospy.sleep(0.5)
            w = 3
            k = random.randint(0, 1)
            if k:
                angle, x, y = PARAMS_PI2P
                p = (p + 1) % 4
            else:
                angle, x, y = PARAMS_PI2M
                p = (p - 1) % 4
            confs = np.array(list(map(int, sys.argv[3:7])))
            case = CONFIGURATIONS[tuple(np.roll(confs, p)[:3])]
            rospy.loginfo("confs " + str(confs))
            rospy.loginfo("Start rotation on angle " + str(angle) + " with velocity " + str(w))
            start_stm_cmd(movement_id + ' 162 ' + ' '.join(map(str, [x, y, angle, abs(x / angle) * w, abs(y / angle) * w, w])))
            rospy.loginfo("Start move to heap configuration " + str(case))
            start_move_cmd(movement_id + " MOVETOHEAP " + str(case))



