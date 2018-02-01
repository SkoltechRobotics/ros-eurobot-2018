#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import numpy as np
import time

class kalman_filter():
    def __init__(self, initial_coords = np.zeros(3), sigma_acc = 20, sigma_acc_a = 0.01, sigma_measure_x = 0.03, sigma_measure_a = 0.01):
        self.speed = np.matrix([0, 0, 0]).T

        self.sigma_acc = sigma_acc
        self.sigma_acc_a = sigma_acc_a
        self.sigma_measure_x = sigma_measure_x
        self.sigma_measure_a = sigma_measure_a
        # measurement covariance matrix:
        self.R = np.matrix(np.diag((self.sigma_measure_x**2, self.sigma_measure_x**2, self.sigma_measure_a**2)))

        self.H = np.matrix([[1,0,0,0,0,0],[0,0,1,0,0,0],[0,0,0,0,1,0]])
        self.Ht = self.H.T
        self.I = np.matrix(np.eye(6))

        self.Xp = np.matrix(np.zeros((6,1)))
        self.Xp[0] = initial_coords[0]
        self.Xp[2] = initial_coords[1]
        self.Xp[4] = initial_coords[2]
        self.Pp = np.matrix(np.eye(6)*1e4)

        # initial iteration to avoid checking on it
        self.t_prev = time.clock()
        self.reset_T()
        self.update(self.Xp[::2,:].copy())

        self.t_prev = time.clock()

    def reset_T(self):
        now = time.clock()
        T = now - self.t_prev
        self.t_prev = now
        self.F = np.matrix([[1, T, 0, 0, 0, 0], [0, 1, 0, 0, 0, 0], [0, 0, 1, T, 0, 0], [0, 0, 0, 1, 0, 0], [0, 0, 0, 0, 1, T], [0, 0, 0, 0, 0, 1]])
        self.Ft = self.F.T
        self.G = np.matrix([[T**2/2, 0, 0], [T, 0, 0], [0, T**2/2, 0], [0, T, 0], [0, 0, T**2/2], [0, 0, T]])
        self.Gt = self.G.T 
        # process covariance matrix: TODO
        self.Q = self.G * self.Gt * self.sigma_acc**2
        self.Q[:6] *= self.sigma_acc**2
        self.Q[6:] *= self.sigma_acc_a**2
        self.B = np.matrix([[T,0,0], [0,0,0], [0,T,0], [0,0,0], [0,0,T], [0,0,0]])

    def predict(self):
        #self.Xf[1,0] = self.speed[0,0]
        #self.Xf[3,0] = self.speed[1,0]
        #self.Xf[5,0] = self.speed[2,0]
        self.Xp = self.F * self.Xf + self.B * self.speed
        self.Pp = self.F * self.Pf * self.Ft + self.Q

    def update(self, Zm):
        self.K = self.Pp * self.Ht * ((self.H * self.Pp * self.Ht) + self.R).I
        self.Xf = self.Xp + self.K * (Zm - self.H * self.Xp)
        self.Pf = (self.I - self.K * self.H) * self.Pp

    def iterate(self, z):
        t1 = time.clock()
        Zm = np.matrix(z).T
        self.reset_T()
        self.predict()
        self.update(Zm)
        #print 'kalman iteration took', time.clock() - t1, 'sec'
        return self.Xf

    def set_speed(self, v):
        #self.speed = np.matrix(v).T
        self.speed = v

def coords_input_callback(data):
    z = map(float, data.data.split())
    X = k_filter.iterate(z)
    pub.publish(' '.join(map(str, [X[0,0], X[2,0], X[4,0]])))

def speed_input_callback(data):
    v = np.matrix(map(float, data.data.split())).T
    angle = k_filter.Xf[5,0]
    M = np.matrix([[np.cos(angle), -np.sin(angle), 0], [np.sin(angle), np.cos(angle), 0], [0, 0, 1]])
    v = M * v
    v[:2] *= 1000
    k_filter.set_speed(v)

if __name__ == '__main__':
    try:
        # create a KF object with params from ROS 
        initial_coords = np.zeros(3)
        initial_coords[0] = rospy.get_param("start_x")
        initial_coords[1] = rospy.get_param("start_y")
        initial_coords[2] = rospy.get_param("start_a")
        k_filter = kalman_filter(initial_coords=initial_coords)

        # ROS entities
        rospy.init_node('kalman_filter', anonymous=True)
        rospy.Subscriber("particle_filter/coordinates", String, coords_input_callback, queue_size=1)
        rospy.Subscriber("track_regulator/speed", String, speed_input_callback, queue_size=1)
        pub = rospy.Publisher('kalman_filter/coordinates', String, queue_size=1)

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()
    except rospy.ROSInterruptException:
                pass

