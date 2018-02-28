#!/usr/bin/env python
import rospy
import numpy as np
#from move_base_msgs.msg import MoveBaseActionGoal
from nav_msgs.msg import Path 
import tf
from scipy.optimize import fminbound
from geometry_msgs.msg import Twist
from sklearn.preprocessing import normalize


V = 0.2
MAX_W = 2.7
# TODO add max accelerations
D = 50
FAR = 0.05
CLOSE = 0.01


def plan_callback(plan):
    global path
    path = np.array([[pose.pose.position.x, pose.pose.position.y, tf.transformations.euler_from_quaternion([0,0,pose.pose.orientation.z, pose.pose.orientation.w])[2]] for pose in plan.poses])
    follow_path()


def follow_path():
    global path
    closest = 0
    def func(x):
        return np.sum((path[int(x),:2] - coords[:2])**2)
    while not rospy.is_shutdown():
        closest = int(fminbound(func, 0, path.shape[0] - 1))
        if func(closest) > FAR or np.sum((path[-1][:2] - coords[:2])**2) < CLOSE ** 2:
            send_cmd(0, 0, 0)
            break
        carrot = min(closest + D, path.shape[0] - 1)
        vec = path[carrot,:] - coords
        print '------------------------'
        print 'coords:', coords
        print 'carrot:', path[carrot,:]
        print 'vec:   ', vec
        vel = rotation_transform(vec, -coords[2])
        print 'vel:   ', vel
        vel[:2] *= V / (vel[0]**2 + vel[1]**2) ** .5 # TODO dynamic velocity (slow down when close to end or obstacles)
        vel[2] = (vel[2] + np.pi) % (2 * np.pi) - np.pi
        if abs(vel[2]) > MAX_W:
            vel[2] *= MAX_W / abs(vel[2])
        print 'finvel:', vel
        send_cmd(*vel)
        rospy.sleep(0.05)
    send_cmd(0, 0, 0)


def rotation_transform(vec, angle):
    M = np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])
    ans = vec.copy()
    ans[:2] = np.matmul(M, ans[:2].reshape((2,1))).reshape(2)
    return ans


def send_cmd(vx, vy, wz):
    tw = Twist()
    tw.linear.x = vx
    tw.linear.y = vy
    tw.angular.z = wz
    cmd_pub.publish(tw)


if __name__ == "__main__":
    rospy.init_node("follower", anonymous=True)
    rospy.Subscriber("move_base/GlobalPlanner/plan", Path, plan_callback, queue_size = 1)
    cmd_pub = rospy.Publisher("/main_robot/cmd_vel", Twist, queue_size = 1)
    listener = tf.TransformListener()
    rate = rospy.Rate(20.0)
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/map', '/main_robot', rospy.Time(0))
            yaw = tf.transformations.euler_from_quaternion(rot)[2]
            coords = np.array([trans[0], trans[1], yaw])
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

    #rospy.spin()
