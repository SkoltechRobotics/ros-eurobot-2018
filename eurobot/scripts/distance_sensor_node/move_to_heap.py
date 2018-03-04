#!/usr/bin/env python
import rospy
import sys
from std_msgs.msg import String
import numpy as np

L = 58
L2 = 107 / 2

A_R = np.zeros((12, 3, 5))
A_R[0] = A_R[2] = np.array([[-0.5, 0, 0, 0, 0.5],
                            [0, -1. / 3, -1. / 3, -1. / 3, 0],
                            [0, -1. / 2 / L2, 0, 1. / 2 / L2, 0]])

A_R[1] = A_R[4] = np.array([[-0.5, 0, 0, 0, 0.5],
                            [0, 0, -1, 0, 0],
                            [0, 0, -1. / L2, 1. / L2, 0]])

A_R[3] = A_R[6] = np.array([[-0.5, 0, 0, 0, 0.5],
                            [0, 0, -1, 0, 0],
                            [0, -1. / L2, 1. / L2, 0, 0]])

A_R[5] = A_R[10] = np.array([[-0.5, 0, 0, 0, 0.5],
                            [0, 0, -1, 0, 0],
                            [0, 0, 0, 0, 0]])

A_R[7] = np.array([[-0.5, 0, 0, 0, 0.5],
                   [0, -1, 0, 0, 0],
                   [0, 0, 0, 0, 0]])

A_R[9] = np.array([[-0.5, 0, 0, 0, 0.5],
                   [0, 0, 0, -1, 0],
                   [0, 0, 0, 0, 0]])

A_R[8] = np.array([[0, 0, 0, 0, 0],
                   [0, 0, -1, 0, 0],
                   [0, 0, 0, 0, 0]])

A_R[11] = np.array([[-0.5, 0, 0, 0, 0.5],
                   [0, 0, 0, 0, 0],
                   [0, 0, 0, 0, 0]])

PLANES = np.array([[0, 0, 0, 0, 0],
                   [1, 1, 0, 0, 0],
                   [0, 0, 1, 0, 0],
                   [0, 0, 0, 1, 1],
                   [1, 1, 1, 0, 0],
                   [1, 1, 0, 1, 1],
                   [0, 0, 1, 1, 1],
                   [2, 1, 2, 0, 0],
                   [3, 1, 0, 1, 3],
                   [0, 0, 2, 1, 2],
                   [1, 1, 1, 1, 1],
                   [0.5, 1, 1, 1, 0.5]])


def fun(r0s, rs, planes):
    return L * planes + r0s - rs


def distance_sensors_callback(data):
    global sensors
    global start_sensors
    a = 0.8
    try:
        sensors1 = np.array(map(int, data.data.split()))
    except ValueError:
        sensors1 = start_sensors

    sensors = a * sensors1 + (1 - a) * sensors


def command_callback(data):
    global sensors
    global start_sensors
    data_splitted = data.data.split()
    action_type = data_splitted[1]
    print("Receive command " + data.data)

    if action_type == "MOVETOHEAP":
        config = int(data_splitted[2])
        while not rospy.is_shutdown():
            f = fun(start_sensors, sensors, PLANES[config])
            x = -A_R[config].dot(f[:, np.newaxis])[:, 0]
            x[0:2] /= -1000
            print(x)
            dt = 0.3
            v = x / dt
            pub_command.publish("MOVE 8 " + ' '.join(map(str, v)))
            rate.sleep()
            if np.all(np.abs(x) < np.array([0.003, 0.003, 0.01])):
                pub_command.publish("MOVE 8 0 0 0")
                break
        pub_response.publish(data_splitted[0] + " finished")


if __name__ == '__main__':
    try:
        sensors = np.zeros(5)
        start_sensors = np.array(list(map(int, sys.argv[1:])))
        rospy.init_node('read_data_node', anonymous=True)
        rate = rospy.Rate(20)

        pub_command = rospy.Publisher("/main_robot/stm_command", String, queue_size=10)
        rospy.Subscriber("/main_robot/move_command", String, command_callback)
        rospy.Subscriber("/distance_sensors/distances", String, distance_sensors_callback)
        pub_response = rospy.Publisher("/main_robot/response", String, queue_size=2)

        rospy.spin()
    except rospy.ROSInterruptException:
        pass
