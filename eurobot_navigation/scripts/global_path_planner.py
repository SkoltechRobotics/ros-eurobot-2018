#!/usr/bin/env python
import numpy as np
import rospy
import rospkg
import tf
from eurobot_navigation.srv import PlanGlobal
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import scipy.interpolate as si

class A_star:

    def __init__(self, data):
        self.data = data
        self.n_x, self.n_y, self.n_a = data.shape

        # ROS
        rospy.init_node('global_path_planner', anonymous=True)
        self.s = rospy.Service('global_path_planner/plan', PlanGlobal, self.handle_plan_global)
        self.pub_path = rospy.Publisher('path', Path, queue_size=1)

    def handle_plan_global(self, req):
        A_a_degrees = tf.transformations.euler_from_quaternion([req.A.pose.orientation.x, req.A.pose.orientation.y,
                                                       req.A.pose.orientation.z, req.A.pose.orientation.w])[2] * (180/np.pi)
        B_a_degrees = tf.transformations.euler_from_quaternion([req.B.pose.orientation.x, req.B.pose.orientation.y,
                                                       req.B.pose.orientation.z, req.B.pose.orientation.w])[2] * (180/np.pi)
        A = (int(req.A.pose.position.x * 1000 / 25), int(req.A.pose.position.y * 1000 / 25), int(A_a_degrees / 10))
        B = (int(req.B.pose.position.x * 1000 / 25), int(req.B.pose.position.y * 1000 / 25), int(B_a_degrees / 10))
        #print A, B
        path = self.search(A, B)
        path = np.array(path)
        # To (mm,mm,rad)
        x = path[:, 0] * 25. / 1000.
        y = path[:, 1] * 25. / 1000.
        a = path[:, 2] * np.pi / 18.
        x, y, a = self.bspline_planning(x, y, a, len(x) * 10)
        path = self.arr2msg(x, y, a)

        self.pub_path.publish(path)

        return path

    def search(self, A, B):
        """Finds path from point A to point B in 3D configuration space (x,y,a) using A-star algorithm.
        A, B - tuples of 3 values."""
        path = []
        self.f = np.full((self.n_x, self.n_y, self.n_a), np.inf)
        self.g = np.full((self.n_x, self.n_y, self.n_a), np.inf)
        parent_x = np.zeros((self.n_x, self.n_y, self.n_a), dtype=int)
        parent_y = np.zeros((self.n_x, self.n_y, self.n_a), dtype=int)
        parent_a = np.zeros((self.n_x, self.n_y, self.n_a), dtype=int)
        self.g[A] = 0
        self.f[A] = self.H(A, B)
        l = [A]
        while len(l) != 0:
            current = self.get_smallest_f(l)
            l.remove(current)
            if current == B:
                return self.path(A, B, parent_x, parent_y, parent_a)
            for n in self.neighbors(current):
                if self.g[n] > self.g[current] + 1:
                    self.g[n] = self.g[current] + 1
                    self.f[n] = self.g[n] + self.H(n, B)
                    parent_x[n], parent_y[n], parent_a[n] = current
                    if n not in l:
                        l.append(n)

    def H(self, P, B):
        """Heuristic function"""
        return abs(P[0] - B[0]) + abs(P[1] - B[1]) + 10 * min(abs(P[2] - B[2]), 360 - abs(P[2] - B[2]))

    def get_smallest_f(self, l):
        smallest = l[0]
        for m in l:
            if self.f[m] < self.f[smallest]:
                smallest = m
        return smallest

    def neighbors(self, P):
        if P[2] == 0:
            p2m = self.n_a - 1
        else:
            p2m = P[2] - 1

        if P[2] == self.n_a - 1:
            p2p = 0
        else:
            p2p = P[2] + 1

        n = [(P[0] - 1, P[1] - 1, P[2]), (P[0] - 1, P[1], P[2]), (P[0] - 1, P[1] + 1, P[2]),
             (P[0], P[1] + 1, P[2]), (P[0] + 1, P[1] + 1, P[2]), (P[0] + 1, P[1], P[2]),
             (P[0] + 1, P[1] - 1, P[2]), (P[0], P[1] - 1, P[2]),

             (P[0] - 1, P[1] - 1, p2m), (P[0] - 1, P[1], p2m), (P[0] - 1, P[1] + 1, p2m),
             (P[0], P[1] - 1, p2m), (P[0], P[1], p2m), (P[0], P[1] + 1, p2m),
             (P[0] + 1, P[1] - 1, p2m), (P[0] + 1, P[1], p2m), (P[0] + 1, P[1] + 1, p2m),

             (P[0] - 1, P[1] - 1, p2p), (P[0] - 1, P[1], p2p), (P[0] - 1, P[1] + 1, p2p),
             (P[0], P[1] - 1, p2p), (P[0], P[1], p2p), (P[0], P[1] + 1, p2p),
             (P[0] + 1, P[1] - 1, p2p), (P[0] + 1, P[1], p2p), (P[0] + 1, P[1] + 1, p2p), ]

        return [i for i in n if i[0] >= 0 and i[0] < self.n_x and i[1] >= 0 and i[1] < self.n_y and self.data[i] == 0]

    def path(self, A, B, parent_x, parent_y, parent_a):
        P = B
        path = [P]
        while P != A:
            P = (parent_x[P], parent_y[P], parent_a[P])
            path.append(P)
        return path

    def arr2msg(self, x, y, a):
        poses = []
        for i in range(len(x)):
            poseP = PoseStamped()
            poseP.pose.position.x = x[i]
            poseP.pose.position.y = y[i]
            orient = tf.transformations.quaternion_from_euler(0, 0, a[i])
            poseP.pose.orientation.z = orient[2]
            poseP.pose.orientation.w = orient[3]
            poses.append(poseP)
        path = Path()
        path.header.frame_id = 'world'
        path.poses = poses
        return path

    def bspline_planning(self, x, y, a, sn):
        N = 3  # B Spline order
        t = range(len(x))
        x_tup = si.splrep(t, x, k=N)
        y_tup = si.splrep(t, y, k=N)
        a_tup = si.splrep(t, a, k=N)

        x_list = list(x_tup)
        xl = x.tolist()
        x_list[1] = xl + [0.0, 0.0, 0.0, 0.0]

        y_list = list(y_tup)
        yl = y.tolist()
        y_list[1] = yl + [0.0, 0.0, 0.0, 0.0]

        a_list = list(a_tup)
        al = a.tolist()
        a_list[1] = al + [0.0, 0.0, 0.0, 0.0]

        ipl_t = np.linspace(0.0, len(x) - 1, sn)
        rx = si.splev(ipl_t, x_list)
        ry = si.splev(ipl_t, y_list)
        ra = si.splev(ipl_t, a_list)

        return rx, ry, ra


if __name__ == '__main__':
    conf_space = np.load('%s/map/conf_space.npy' % rospkg.RosPack().get_path('eurobot_navigation'))
    alg = A_star(conf_space)

    rospy.spin()