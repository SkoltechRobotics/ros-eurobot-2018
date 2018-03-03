#!/usr/bin/env python
import rospy
import sys
from std_msgs.msg import String
import numpy as np
from numpy import cos, tan

L = 58
L2 = 50
L3 = 72
A_R = np.array([[-0.5, 0, 0, 0.5],
                [0, -0.5, -0.5, 0],
                [0, -1. / (L2 + L3), 1./(L2 + L3), 0]])


A_R[2] = np.array([[-0.5, 0, 0, 0.5],
                  [0, 0, -1, 0],
                  [0, 0, 0, 0]])

A_R[3] = np.array([[-0.5, 0, 0, 0.5],
                  [0, 0, 0, 0],
                  [0, 0, 0, 0]])

A_R[4] = np.array([[0, 0, 0, 0],
                   [0, 0, 0, 0],
                   [0, 0, 0, 0]])

PLANES = [[0, 0, 0, 0],
          [1, 1, 0, 0],
          [0, 0, 0, 0],
          [0, 0, 1, 1],
          [1, 1, 0, 0],
          [1, 1, 1, 1],
          [0, 0, 1, 1],
          [2, 1, 0, 0],
          [3, 1, 1, 3],
          [0, 0, 1, 2],
          [1, 1, 1, 1]]

MATRICES = [0, 2, 0, 1, 2, 3, 1, 2, 4, 1, 3]


def distances(point, planes):
    x_0, y_0, alpha_0 = point
    dr = np.zeros(4)
    dr[0] = 3 * L / 2 - 3 * L / (2 * cos(alpha_0)) - x_0 - y_0 * tan(alpha_0)
    dr[1] = L / 2 - L / (2 * cos(alpha_0)) - L2 * tan(alpha_0) + x_0 * tan(alpha_0) - y_0
    dr[2] = L / 2 - L / (2 * cos(alpha_0)) + L3 * tan(alpha_0) + x_0 * tan(alpha_0) - y_0
    dr[3] = 3 * L / 2 - 3 * L / (2 * cos(alpha_0)) + x_0 + y_0 * tan(alpha_0)
    return dr


def fun(x, r0s, rs):
    print r0s, rs
    return distances(x) + r0s - rs


def command_callback(data):
    global sensors
    global start_sensors
    data_splitted = data.data.split()
    action_type = data_splitted[1]
    print("Receive command " + data.data)

    if action_type == "MOVETOHEAP":
        # vm = np.array([0.3, 0.3, 0.5])
        # x_last = np.array([0, 0, 0])
        while not rospy.is_shutdown():
            x = np.array([0, 0, 0])
            for i in range(1):
                f = fun(x, start_sensors, sensors, PLANES[config])
                dX = A_R[MATRICES[config]].dot(f[:, np.newaxis])[:, 0]
                x = x - dX
            x[0:2] /= -1000
            print(x)
            # dx = x - x_last
            # x_last = x
            dt = 0.3
            v = x / dt
            pub_command.publish("MOVE 8 " + ' '.join(map(str, v)))
            rate.sleep()
            if np.all(np.abs(x) < np.array([0.003, 0.003, 0.01])):
                pub_command.publish("MOVE 8 0 0 0")
                break
        pub_response.publish(data_splitted[0] + " finished")


def distance_sensors_callback(data):
    global sensors
    global start_sensors
    a = 0.8
    try:
        sensors1 = np.array(map(int, data.data.split()))
    except ValueError:
        sensors1 = start_sensors

    sensors = a * sensors1 + (1 - a) * sensors


if __name__ == '__main__':
    try:
        sensors = np.zeros(4)
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
