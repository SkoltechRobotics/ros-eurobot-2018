#!/usr/bin/env python
import rospy
import sys
from std_msgs.msg import String
import numpy as np
from std_msgs.msg import Float32MultiArray
import time

KP = np.array([1, 1, 0.5])
KD = np.array([0.3, 0.3, 0.3])
MAX_VELOCITY = np.array([0.3, 0.3, 0.5])
DT = 1. / 20
SIGMA_V = np.array([0.01, 0.01, 0.01])
SIGMA_X = np.array([0.003, 0.003, 0.03])


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
                   [0, 0, 0, -1, 0],
                   [0, 0, 0, 0, 0]])

A_R[9] = np.array([[-0.5, 0, 0, 0, 0.5],
                   [0, -1, 0, 0, 0],
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


class PIDRegulator(object):
    def __init__(self, k_p, k_d, k_i, max_response, max_integral):
        self.k_p = k_p
        self.k_d = k_d
        self.k_i = k_i
        self.max_response = max_response
        self.max_integral = max_integral
        self.target = np.zeros_like(k_p)
        self.error_i = np.zeros_like(k_p)
        self.prev_error = np.zeros_like(k_p)
        self.prev_time = time.time()

    def set_target(self, target):
        self.target = target

    def regulate(self, feedback):
        dt = time.time() - self.prev_time
        self.prev_time = time.time()

        error_p = self.target - feedback

        self.error_i += error_p * dt
        error_i = self.error_i

        error_d = (error_p - self.prev_error) / dt

        self.prev_error = error_p

        self.error_i = np.where(self.error_i > -self.max_integral, self.error_i, -self.max_integral)
        self.error_i = np.where(self.error_i < self.max_integral, self.error_i, self.max_integral)
        print(error_p, error_d, error_i, self.k_p, self.k_d, self.k_i)
        response = error_p * self.k_p + error_i * self.k_i + error_d * self.k_d
        response = np.where(response > -self.max_response, response, -self.max_response)
        response = np.where(response < self.max_response, response, self.max_response)
        return response


# class Kalman(object):
#     def __init__(self, x0, p0, matrix_t, matrix_h, matrix_r, matrix_q):
#         self.x = x0
#         self.p = p0
#         self.t = matrix_t
#         self.h = matrix_h
#         self.r = matrix_r
#         self.q = matrix_q
#
#     def iteration(self, z):
#         # Prediction
#         self.x = self.t.dot(self.x[:, np.newaxis]).T[0]
#         self.p = self.t.dot(self.p).dot(self.t.T) + self.q
#         # Filtration
#         tmp1 = np.linalg.inv(self.h.dot(self.p.dot(self.h.T)) + self.r)
#         tmp2 = self.h.T.dot(tmp1)
#         k = self.p.dot(tmp2)
#         self.x = self.x + k * (z - self.h.dot(self.x))
#         self.p = (np.eye(self.x.shape[0]) - k.dot(self.h)).dot(self.p)
#         return self.x


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
    rospy.loginfo("Receive command " + data.data)

    if action_type == "MOVETOHEAP":
        config = int(data_splitted[2])
        rospy.sleep(0.5)
        rospy.loginfo("Start move to heap by rangefinders")
        pid = PIDRegulator(KP, KD, np.zeros(3), MAX_VELOCITY, np.zeros(3))
        pid.set_target(np.zeros(3))

        # # Init Kalman
        # T = np.eye(6)
        # T[3:, :3] = np.eye(3) * DT
        # H = np.eye(6)
        # R = np.eye(6)
        # R[:3, :3] *= SIGMA_X ** 2
        # R[3:, 3:] *= SIGMA_V ** 2
        # G = np.ones(6)
        # G[:3] *= DT
        # Q = G[:, np.newaxis].dot(G[np.newaxis, :]) * SIGMA_V ** 2
        # kalman = Kalman()
        while not rospy.is_shutdown():
            f = fun(start_sensors, sensors, PLANES[config])
            x = -A_R[config].dot(f[:, np.newaxis])[:, 0]
            x[0:2] /= -1000
            pub_movement.publish(Float32MultiArray(data=x))
            v = pid.regulate(x)
            pub_command.publish("MOVE 8 " + ' '.join(map(str, v)))
            rate.sleep()
            if np.all(np.abs(x) < np.array([0.001, 0.001, 0.01])):
                pub_command.publish("MOVE 8 0 0 0")
                pub_response.publish(data_splitted[0] + " finished")
                rospy.loginfo("MOVETOHEAP finished")
                break
            if np.any(np.abs(x) > np.array([0.04, 0.04, 0.4])):
                pub_command.publish("MOVE 8 0 0 0")
                rospy.logerr("MOVETOHEAP failed")
                break
                
        pub_command.publish("MOVE 8 0 0 0")


if __name__ == '__main__':
    try:
        sensors = np.zeros(5)
        start_sensors = np.array(list(map(float, sys.argv[1:])))
        rospy.init_node('read_data_node', anonymous=True)
        rate = rospy.Rate(20)

        pub_command = rospy.Publisher("/main_robot/stm_command", String, queue_size=10)
        rospy.Subscriber("/main_robot/move_command", String, command_callback)
        rospy.Subscriber("/distance_sensors/distances/smooth", Float32MultiArray, distance_sensors_callback)
        pub_response = rospy.Publisher("/main_robot/response", String, queue_size=2)
        pub_movement = rospy.Publisher("/main_robot/rangefinder_movement", Float32MultiArray, queue_size=2)

        rospy.spin()
    except rospy.ROSInterruptException:
        pass
