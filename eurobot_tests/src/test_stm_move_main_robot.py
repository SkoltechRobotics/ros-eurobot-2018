#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import sys
import time
import random

PI = 3.1415926
PARAMS_PI2P = [1.019 * PI / 2, 0.096, 0.014]
PARAMS_PI2M = [-1.02 * PI / 2, -0.096, 0.01]
# (number of repeats) rotation (rotation velocity)


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


if __name__ == '__main__':
    rospy.init_node("test_stm_move_main_robot")
    pub_stm = rospy.Publisher("/main_robot/stm_command", String, queue_size=10)
    rospy.Subscriber("/main_robot/response", String, response_callback)
    rate = rospy.Rate(100)
    rospy.loginfo("Start test move stm move")
    rospy.sleep(1.5)

    is_response = False
    movement_id = "test_stm_main_robot"
    n = int(sys.argv[1])

    for i in range(n):
        if rospy.is_shutdown():
            break
        if sys.argv[2] == "rotation":
            rospy.sleep(0.5)
            w = float(sys.argv[3])
            if sys.argv[4] == "random":
                k = random.randint(-2, 2)
            else:
                k = float(sys.argv[4])

            angle = PI / 2 * k
            rospy.loginfo("Start rotation on angle " + str(angle) + " with velocity " + str(w))
            x = float(sys.argv[5])
            y = float(sys.argv[6])
            start_cmd(movement_id + ' 162 ' + ' '.join(map(str, [x, y, angle, abs(x / angle) * w, abs(y / angle) * w, w])))
        if sys.argv[2] == "linear":
            rospy.sleep(0.5)
            v = float(sys.argv[3])
            k = (-1) ** i

            dist_x = float(sys.argv[4]) * k
            dist_y = float(sys.argv[5]) * k
            rospy.loginfo("Start linear movement on distance " + str(dist_x) + " " + str(dist_y) + " with velocity " +
                          str(v))
            dist = (dist_x ** 2 + dist_y ** 2) ** 0.5

            start_cmd(movement_id + ' 162 ' + ' '.join(map(str, [dist_x, dist_y, 0, abs(dist_x) / dist * v,
                                                                abs(dist_y) / dist * v, 0])))
        if sys.argv[2] == "new_rotation":
            rospy.sleep(0.5)
            w = 3
            k = random.randint(0, 1)
            if k:
                angle, x, y = PARAMS_PI2P
            else:
                angle, x, y = PARAMS_PI2M

            rospy.loginfo("Start rotation on angle " + str(angle) + " with velocity " + str(w))
            start_cmd(movement_id + ' 162 ' + ' '.join(map(str, [x, y, angle, abs(x / angle) * w, abs(y / angle) * w, w])))
