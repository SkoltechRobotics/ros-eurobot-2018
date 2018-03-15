#!/usr/bin/env python
import rospy
import numpy as np
from nav_msgs.msg import Path 
import tf
from scipy.optimize import fminbound
from geometry_msgs.msg import Twist
from sklearn.preprocessing import normalize
from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseActionGoal


V_MAX = 0.2
V_SLOW = 0.01
W_MAX = 2.7
ACCELERATION = np.array([3, 3, 6])
# TODO add max accelerations ?
D = 20
D_ACCELERATION = 10
D_DECELERATION = 15
FAR = 0.07
XY_GOAL_TOLERANCE = 0.001
YAW_GOAL_TOLERANCE = 0.05
goal_id = ''
heap_approaching = False
goal_point = np.zeros(3)
via_point = np.zeros(3)

approaching_distance = 0.1
heap_radius = 0.092
path_pixel = 0.005


def plan_callback(plan):
    path = np.array([[pose.pose.position.x, pose.pose.position.y, tf.transformations.euler_from_quaternion([0,0,pose.pose.orientation.z, pose.pose.orientation.w])[2]] for pose in plan.poses])
    global t_prev
    t_prev = rospy.get_time()

    # extrapolate path if approaching a heap
    global heap_approaching
    if heap_approaching == True:
        length = np.linalg.norm(goal_point[:2] - via_point[:2])
        n_extra_points = int(length / path_pixel)
        extra_path = np.zeros((n_extra_points, 3))
        extra_path[:,0] = np.linspace(via_point[0], goal_point[0], num=n_extra_points)
        extra_path[:,1] = np.linspace(via_point[1], goal_point[1], num=n_extra_points)
        extra_path[:,2] = np.ones(n_extra_points) * goal_point[2]
        path = np.concatenate((path, extra_path), axis=0)
        heap_approaching = False

    print np.array([np.linalg.norm(path[i+1][:2] - path[i][:2]) for i in range(path.shape[0] - 1)])
    
    rospy.loginfo('Recieved a path to ' + str(path[-1]))
    follow_path(path)
    rospy.loginfo('Arrived to ' + str(path[-1]))


def follow_path(path):
    # indexes of the closest path point and the last (goal) path point
    closest = 0
    goal = path.shape[0] - 1
    
    # A function for fminbound algorithm to seek the closest path point index
    def func(x):
        """Distance from path point with index x to robot coords in metric L1."""
        return np.sum((path[int(x),:2] - coords[:2])**2) ** .5

    while not rospy.is_shutdown():
        t0 = rospy.get_time()
        #rospy.loginfo('STARTED NEW ITERATION')
        # current linear and angular goal distance
        goal_distance = func(goal)
        goal_yaw_distance = abs(path[-1][2] - coords[2])
        #rospy.loginfo('goal_distance = ' + str(goal_distance) + ' ' + str(goal_yaw_distance))

        # find index of the closest path point by solving an optimization problem
        closest = int(fminbound(func, 0, goal))
        path_deviation = func(closest)
        #rospy.loginfo('closest: ' + str(closest))
        #rospy.loginfo('path_deviation: ' + str(path_deviation))

        # stop and publish response if we reached the goal with the given tolerance
        if path_deviation > FAR or (goal_distance < XY_GOAL_TOLERANCE and goal_yaw_distance < YAW_GOAL_TOLERANCE):
            # stop the robot
            speed_cmd(0, 0, 0)
            pub_response.publish(goal_id + " finished")
            #if path_deviation > FAR:
            #    rospy.loginfo(goal_id + " terminated, path deviation")
            #else:
            #    rospy.loginfo(goal_id + " finished, reached the goal")
            break
        #rospy.loginfo('funk(closest) = ' + str(closest))

        # place a carrot on the path for the robot to follow (it is D steps ahead of the robot)
        carrot = min(closest + D, goal)
        #rospy.loginfo('carrot = ' + str(carrot))

        # VELOCITY REGULATION.
        # Here we assume W * T << 1, where W is angular speed and T is the time of one iteration of the control loop. Otherwice the path of the robot during one iteration will be an arc, not a line.
        # distance to the carrot
        carrot_distance = path[carrot, :] - coords
        carrot_distance[2] = (carrot_distance[2] + np.pi) % (2 * np.pi) - np.pi
        #rospy.loginfo('carrot_distance:\t' + str(carrot_distance))

        # choose speed limits
        # deceleration in the end of the path
        acceleration_coefficient = min(1.0, 0.2 + float(closest) / D_ACCELERATION)
        deceleration_coefficient = min(1.0, float(goal-closest) / D_DECELERATION)
        #rospy.loginfo('acc, dec coefficients = ' + str(acceleration_coefficient) + ' ' + str(deceleration_coefficient))
        
        # maximum possible speed in carrot distance proportion
        vel = V_MAX * carrot_distance / np.max(np.abs(carrot_distance[:2]))
        if abs(vel[2]) > W_MAX:
            vel *= W_MAX / abs(vel[2])
        #rospy.loginfo('vel max\t:' + str(vel))

        # consider acceleration and deceleration
        vel = vel * deceleration_coefficient * acceleration_coefficient
        #rospy.loginfo('vel*coef\t:' + str(vel))

        # vel in robot frame
        vel_robot_frame = rotation_transform(vel, -coords[2])
        #rospy.loginfo('vel cmd\t:' + str(vel_robot_frame))
        
        speed_cmd(*vel_robot_frame)

        t = rospy.get_time()
        global t_prev
        #rospy.loginfo('>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> Time (sec)  of  iteration: ' + str(t - t0))
        #rospy.loginfo('>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> Time (sec) betw iteration: ' + str(t - t_prev))
        t_prev = t
        # for debug
        #rospy.loginfo('------------------------')
        if t - t0 < 0.05:
            rospy.sleep(0.05)
    # stop the robot in case the node is closed
    speed_cmd(0, 0, 0)

def rotation_transform(vec, angle):
    M = np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])
    ans = vec.copy()
    ans[:2] = np.matmul(M, ans[:2].reshape((2,1))).reshape(2)
    return ans


def speed_cmd(vx, vy, wz):
    tw = Twist()
    tw.linear.x = vx
    tw.linear.y = vy
    tw.angular.z = wz
    pub_twist.publish(tw)


def close_to_heap(goal):
    dist = np.sum((heap_coords - goal) ** 2, axis=1) ** .5
    if np.min(dist) < approaching_distance + heap_radius:
        # return coords of the closest heap if the distance is critical
        return True, heap_coords[np.argmin(dist)]
    return False, []


def cmd_callback(data):
    # parse name,type
    data_splitted = data.data.split()
    cmd_id = data_splitted[0]
    cmd_type = data_splitted[1]
    cmd_args = data_splitted[2:]

    if cmd_type == "move": # movement with navigation
        global goal_point, via_point
        goal_point = np.array([float(cmd_args[0]), float(cmd_args[1]), float(cmd_args[2])])
        
        # check if the goal pose is close to any cube heap
        is_close, closest_coords = close_to_heap(goal_point[:2])
        if is_close:
            global heap_approaching
            heap_approaching = True
            dist = (closest_coords - goal_point[:2])
            if np.sum(dist ** 2) ** .5 < 1e-4:
                # approaching movement diraction
                e = np.array([-np.sin(goal_point[2]), np.cos(goal_point[2])])
            else:
                e = dist / np.sum(dist ** 2) ** .5
            # make a via-point for approaching heap with linear path starting at it
            via_point[:2] = goal_point[:2] - e * (approaching_distance + heap_radius)
            via_point[2] = goal_point[2] # TODO: make robot face the center of the heap (for safety) regardless of the input
        else:
            via_point = goal_point
        # create a message for move_base (to ask for a global plan)
        move_message = MoveBaseActionGoal()
        move_message.goal.target_pose.header.frame_id = 'world'
        move_message.goal.target_pose.pose.position.x = via_point[0]
        move_message.goal.target_pose.pose.position.y = via_point[1]
        
        q = tf.transformations.quaternion_from_euler(0, 0, via_point[2])
        move_message.goal.target_pose.pose.orientation.z = q[2]
        move_message.goal.target_pose.pose.orientation.w = q[3]

        # remember goal_id
        global goal_id
        goal_id = cmd_id
        
        # this is a command to the global planner
        pub_goal.publish(move_message)

    elif cmd_type == "move_odometry": # simple movement by odometry
        #rospy.loginfo('=========================== (sleep)')
        rospy.sleep(1.0)
        #rospy.loginfo('MOVE ODOM')
        goal = np.array(cmd_args).astype('float')
        #rospy.loginfo('goal: ' + str(goal))
        d = rotation_transform((goal[:2] - coords[:2]), -coords[2])
        v = np.abs(d[:2]) / np.abs(np.max(d[:2])) * 0.2
        cmd = cmd_id + " 162 " + str(d[0]) + ' ' + str(d[1]) + ' 0 ' + str(v[0]) + ' ' + str(v[1]) + ' 0'
        #rospy.loginfo('stm cmd: ' + str(cmd))
        pub_cmd.publish(cmd)
        #rospy.loginfo('===========================')

def speeds_proportion_to_reach_point(point): # TODO: check
    """ The speed to reach the point given in robot frame in uniform motion"""
    x, y, a = point
    # radius of the path to the point
    R = np.sqrt(x ** 2 + y ** 2) / 2 / np.abs(np.sin(a / 2))
    angle = np.arctan2(y, x) - a / 2
    vel = np.array([R * np.cos(angle), R * np.sin(angle), 1.0])
    return vel / abs(np.max(vel))


if __name__ == "__main__":
    rospy.init_node("follower", anonymous=True)
    rospy.Subscriber("/move_base/GlobalPlanner/plan", Path, plan_callback, queue_size = 1)
    rospy.Subscriber("/main_robot/move_command", String, cmd_callback, queue_size = 1)
    pub_twist = rospy.Publisher("/main_robot/cmd_vel", Twist, queue_size = 1)
    pub_goal = rospy.Publisher("/move_base/goal", MoveBaseActionGoal, queue_size = 1)
    pub_response = rospy.Publisher("/main_robot/response", String, queue_size=10)
    pub_cmd = rospy.Publisher("main_robot/stm_command", String, queue_size=1)
    
    # get initial cube heap coordinates
    n_heaps = 6
    heap_coords = np.zeros((n_heaps, 2))
    for n in range(n_heaps):
        heap_coords[n,0] = rospy.get_param("/field/cube" + str(n + 1) + "c_x") / 1000
        heap_coords[n,1] = rospy.get_param("/field/cube" + str(n + 1) + "c_y") / 1000

    listener = tf.TransformListener()
    rate = rospy.Rate(20.0)
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/map', '/main_robot', rospy.Time(0))
            yaw = tf.transformations.euler_from_quaternion(rot)[2]
            coords = np.array([trans[0], trans[1], yaw])
            # rospy.loginfo('got new coords: ' + str(coords))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

    #rospy.spin()
