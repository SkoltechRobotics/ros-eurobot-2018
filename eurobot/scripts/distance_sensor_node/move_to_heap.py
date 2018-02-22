#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import numpy as np
from numpy import cos, tan

L = 5.8
L2 = 5
L3 = 7.2
A_R = np.array([[-0.5, 0, 0, 0.5],
                [0, -0.5, -0.5, 0],
                [0, -1 / (L2 + L3), 1/(L2 + L3), 0]])


def distances(point):
    x_0, y_0, alpha_0 = point
    dr = np.zeros(4)
    dr[0] = 3 * L / 2 - 3 * L / (2 * cos(alpha_0)) - x_0 - y_0 * tan(alpha_0)
    dr[1] = L / 2 - L / (2 * cos(alpha_0)) - L2 * tan(alpha_0) + x_0 * tan(alpha_0) - y_0
    dr[2] = L / 2 - L / (2 * cos(alpha_0)) + L3 * tan(alpha_0) + x_0 * tan(alpha_0) - y_0
    dr[3] = 3 * L / 2 - 3 * L / (2 * cos(alpha_0)) + x_0 + y_0 * tan(alpha_0)
    return dr


def fun(x, r0s, rs):
    return distances(x) + r0s - rs


def command_callback(data):
    global sensors
    global start_sensors
    data_splitted = data.data.split()
    action_type = data_splitted[1]
    print("Receive command " + data.data)

    x = [0, 0, 0]
    if action_type == "MOVETOHEAP":
        x = [0, 0, 0]
        for i in range(3):
            f = fun(x, start_sensors, sensors)
            dX = A_R.dot(f[:, np.newaxis])[:, 0]
            x = x - dX


def distance_sensors_callback(data):
    global sensors
    global start_sensors
    a = 0.8
    try:
        sensors1 = map(int, data.data.split())
    except ValueError:
        sensors1 = start_sensors

    sensors = a * sensors1 + (1 - a) * sensors


if __name__ == '__main__':
    try:
        sensors = np.zeros(4)
        start_sensors = np.array([35, 85, 85, 35])
        rospy.init_node('read_data_node', anonymous=True)

        pub_command = rospy.Publisher("/main_robot/stm_command", String, queue_size=10)
        rospy.Subscriber("/main_robot/move_command", String, command_callback)
        rospy.Subscriber("distance_sensors/distances", String, distance_sensors_callback)

        rospy.spin()
    except rospy.ROSInterruptException:
        pass
