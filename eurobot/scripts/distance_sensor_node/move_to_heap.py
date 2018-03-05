#!/usr/bin/env python
import rospy
import sys
from std_msgs.msg import String
import numpy as np
from std_msgs.msg import Float32MultiArray

L = 58
L2 = 117 / 2

A_R = np.zeros((12, 3, 5))
A_R[0] = A_R[2] = np.array([[-0.5, 0, 0, 0, 0.5],
                            [0, -1. / 2, 0, -1. / 2, 0],
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
    sensors = np.array(data.data)


def command_callback(data):
    global sensors
    global start_sensors
    data_splitted = data.data.split()
    action_type = data_splitted[1]
    print("Receive command " + data.data)

    if action_type == "MOVETOHEAP":
        config = int(data_splitted[2])
        rospy.sleep(1)
        while not rospy.is_shutdown():
            f = fun(start_sensors, sensors, PLANES[config])
            x = -A_R[config].dot(f[:, np.newaxis])[:, 0]
            x[0:2] /= -1000
            # rospy.loginfo("x, y, a = " + str(x.round(4)))
            pub_movement.publish(Float32MultiArray(data=x))
            dt = 0.3
            v = x / dt
            pub_command.publish("MOVE 8 " + ' '.join(map(str, v)))
            rate.sleep()
            if np.all(np.abs(x) < np.array([0.003, 0.003, 0.01])):
                pub_command.publish("MOVE 8 0 0 0")
                pub_response.publish(data_splitted[0] + " finished")
                rospy.loginfo("MOVETOHEAP finished")
                break
            if np.any(np.abs(x) > np.array([0.03, 0.03, 0.15])):
                pub_command.publish("MOVE 8 0 0 0")
                rospy.logerr("MOVETOHEAP failed")
                break
                
        pub_command.publish("MOVE 8 0 0 0")


if __name__ == '__main__':
    try:
        sensors = np.zeros(5)
        start_sensors = np.array(list(map(int, sys.argv[1:])))
        rospy.init_node('read_data_node', anonymous=True)
        rate = rospy.Rate(20)

        pub_command = rospy.Publisher("/main_robot/stm_command", String, queue_size=10)
        rospy.Subscriber("/main_robot/move_command", String, command_callback)
        rospy.Subscriber("/distance_sensors/distances/smooth", Float32MultiArray, distance_sensors_callback)
        pub_response = rospy.Publisher("/main_robot/response", String, queue_size=2)
        pub_movement = rospy.Publisher("/main_robot/mov_from_rangefinders", Float32MultiArray, queue_size=2)

        rospy.spin()
    except rospy.ROSInterruptException:
        pass
