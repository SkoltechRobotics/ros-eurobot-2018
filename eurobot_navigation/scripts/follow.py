#!/usr/bin/env python
import rospy
import numpy as np
#from move_base_msgs.msg import MoveBaseActionGoal
from nav_msgs.msg import Path 
import tf
from scipy.optimize import fminbound
from geometry_msgs.msg import Twist
from sklearn.preprocessing import normalize
from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseActionGoal


V = 0.2
V_SLOW = 0.01
MAX_W = 2.7
ACCELERATION = np.array([3, 3, 6])
# TODO add max accelerations ?
D = 20
D_DECELERATE = 20
FAR = 0.05
XY_GOAL_TOLERANCE = 0.01
YAW_GOAL_TOLERANCE = 0.05
goal_id = ''


def goal_callback(goal):
    global goal_id
    goal_id = goal.goal_id.id

def plan_callback(plan):
    path = np.array([[pose.pose.position.x, pose.pose.position.y, tf.transformations.euler_from_quaternion([0,0,pose.pose.orientation.z, pose.pose.orientation.w])[2]] for pose in plan.poses])
    follow_path(path)


def follow_path(path):
    closest = 0
    goal = path.shape[0] - 1
    #if goal == -1:
    #    return
    def func(x):
        return np.sum((path[int(x),:2] - coords[:2])**2)
    while not rospy.is_shutdown():
        dx = np.sum((path[-1][:2] - coords[:2])**2) ** .5
        dw = abs(path[-1][2] - coords[2])
        closest = int(fminbound(func, 0, goal))
        if func(closest) > FAR or (dx < XY_GOAL_TOLERANCE and dw < YAW_GOAL_TOLERANCE):
            pub_response.publish(goal_id + " finished")
            send_cmd(0, 0, 0)
            break
        carrot = min(closest + D, path.shape[0] - 1)
        vec = path[carrot,:] - coords
        print '------------------------'
        #print 'coords:', coords
        #print 'carrot:', path[carrot,:]
        #print 'vec:   ', vec
        vel = rotation_transform(vec, -coords[2])
        print 'vel:   ', vel
        translation_V = V * min(1, (float(goal-closest) / D_DECELERATE)) # TODO
        print 'V:', translation_V
        vel[:2] *= translation_V / (vel[0]**2 + vel[1]**2) ** .5 # TODO dynamic velocity (slow down when close to end or obstacles)
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
    rospy.Subscriber("move_base/goal", MoveBaseActionGoal, goal_callback, queue_size = 1)
    cmd_pub = rospy.Publisher("/main_robot/cmd_vel", Twist, queue_size = 1)
    pub_response = rospy.Publisher("response", String, queue_size=10)

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
