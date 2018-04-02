#!/usr/bin/env python
import rospy
import numpy as np
from tf import TransformListener, LookupException, ConnectivityException, ExtrapolationException
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from scipy.optimize import fminbound
from geometry_msgs.msg import Twist, PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseActionGoal
from nav_msgs.srv import GetPlan
from threading import Lock


class LocalPlanner:
    # maximum linear and rotational speed
    V_MAX = 0.57
    W_MAX = 2.7
    # acceleration of the robot until V_MAX
    ACCELERATION = 1
    # plan resolution
    RESOLUTION = 0.005
    # distance for placing a carrot
    D = 0.1
    # length of acceleration/deceleration tracks
    D_DECELERATION = 0.1
    # on which distance to consider that robot lost it's path
    FOLLOW_TOLERANCE = 0.15
    # goal tolerance
    XY_GOAL_TOLERANCE = 0.001
    YAW_GOAL_TOLERANCE = 0.05
    # goal tolerance for global planner (when goal of requested plan is obstructed)
    GLOBAL_PLAN_TOLERANCE = 0
    # number of cube heaps
    N_HEAPS = 6
    # max rate of the planner
    RATE = 100
    # distance between a via-point and a cube heap when approaching the heap
    HEAP_APPROACHING_DISTANCE = 0.17
    # speed for odometry movements
    V_MAX_ODOMETRY_MOVEMENT = 0.57

    def __init__(self):
        # a Lock is used to prevent mixing bytes of diff commands to STM
        self.mutex = Lock()

        # ROS entities
        rospy.init_node("path_follower", anonymous=True)

        rospy.Subscriber("move_command", String, self.cmd_callback, queue_size=1)
        self.pub_twist = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        self.pub_goal = rospy.Publisher("move_base/goal", MoveBaseActionGoal, queue_size=1)
        self.pub_response = rospy.Publisher("response", String, queue_size=10)
        self.pub_cmd = rospy.Publisher("stm_command", String, queue_size=1)
        self.listener = TransformListener()

        # get ROS params
        self.robot_name = rospy.get_param("robot_name")
        self.coords = np.array([rospy.get_param("/" + self.robot_name + "/start_x"), rospy.get_param("/" + self.robot_name + "/start_y"), rospy.get_param("/" + self.robot_name + "/start_a")])
        self.pose = self.coords2pose(self.coords)
        self.vel = np.zeros(3)
        # TODO: extrapolate pose (because its update rate is low)

        # get initial cube heap coordinates
        self.heap_coords = np.zeros((self.N_HEAPS, 2))
        for n in range(self.N_HEAPS):
            self.heap_coords[n, 0] = rospy.get_param("/field/cube" + str(n + 1) + "c_x") / 1000
            self.heap_coords[n, 1] = rospy.get_param("/field/cube" + str(n + 1) + "c_y") / 1000

        self.plan = np.array([])
        self.plan_length = self.plan.shape[0]
        self.goal_id = ''
        self.t_prev = rospy.get_time()
        self.D_steps = int(self.D / self.RESOLUTION)

        # start the main timer that will follow given paths
        rospy.Timer(rospy.Duration(1.0 / self.RATE), self.next)

    def next(self, event):
        self.mutex.acquire()

        try:
            (trans, rot) = self.listener.lookupTransform('/map', '/' + self.robot_name, rospy.Time(0))
            self.pose = Pose(Point(*trans), Quaternion(*rot))
            self.coords = self.pose2coords(self.pose)
        except (LookupException, ConnectivityException, ExtrapolationException):
            rospy.loginfo("LocalPlanner failed to lookup tf.")
            self.mutex.release()
            return

        if self.plan_length == 0:
            self.mutex.release()
            return

        # FOLLOW THE PLAN
        rospy.loginfo('STARTED NEW ITERATION')
        # current linear and angular goal distance
        goal_distance = self.distance(self.plan_length - 1)
        goal_yaw_distance = abs(self.plan[-1][2] - self.coords[2])
        rospy.loginfo('goal_distance: ' + str(goal_distance) + ' ; ' + str(goal_yaw_distance))

        # find index of the closest path point by solving an optimization problem
        closest = int(fminbound(self.distance, 0., self.plan_length - 1.))
        path_deviation = self.distance(closest)
        rospy.loginfo('closest: ' + str(closest))
        rospy.loginfo('path_deviation: ' + str(path_deviation))

        # stop and publish response if we have reached the goal with the given tolerance
        if (goal_distance < self.XY_GOAL_TOLERANCE and goal_yaw_distance < self.YAW_GOAL_TOLERANCE):
            # stop the robot
            rospy.loginfo(self.goal_id + " finished, reached the goal")
            self.terminate_following(True)
            self.mutex.release()
            return
        if path_deviation > self.FOLLOW_TOLERANCE:
            # stop the robot
            rospy.loginfo(self.goal_id + " terminated, path deviation")
            self.terminate_following(False)
            self.mutex.release()
            return

        # place a carrot on the path for the robot to follow (it is D steps ahead of the robot)
        carrot = min(closest + self.D_steps, self.plan_length - 1)
        rospy.loginfo('carrot = ' + str(carrot))

        # VELOCITY REGULATION.
        # distance to the carrot
        carrot_distance = self.plan[carrot] - self.coords
        carrot_distance[2] = (carrot_distance[2] + np.pi) % (2 * np.pi) - np.pi
        rospy.loginfo('carrot_distance:\t' + str(carrot_distance))

        t0 = rospy.get_time()
        dt = t0 - self.t_prev
        self.t_prev = t0

        # set speed limit
        if goal_distance < self.D_DECELERATION:
            speed_limit_coefficient = goal_distance / self.D_DECELERATION
            rospy.loginfo('speed_limit_coefficient = ' + str(speed_limit_coefficient) + ' (deceleration)')
        else:
            speed_limit_coefficient = min(self.V_MAX, np.linalg.norm(self.vel[:2]) + self.ACCELERATION * dt) / self.V_MAX
            rospy.loginfo('speed_limit_coefficient = ' + str(speed_limit_coefficient) + ' (acceleration)')

        # maximum possible speed in carrot distance proportion
        vel = self.V_MAX * carrot_distance / np.max(np.abs(carrot_distance[:2]))
        if abs(vel[2]) > self.W_MAX:
            vel *= self.W_MAX / abs(vel[2])
        rospy.loginfo('vel max\t\t:' + str(vel))

        # apply speed limit
        vel = vel * speed_limit_coefficient
        rospy.loginfo('vel after speed limit\t:' + str(vel))

        # vel in robot frame
        vel_robot_frame = self.rotation_transform(vel, -self.coords[2])
        rospy.loginfo('vel cmd\t:' + str(vel_robot_frame))

        # send speed cmd to the robot
        self.set_speed(vel_robot_frame)

        # sleep to keep the rate
        if dt < 1. / self.RATE:
            rospy.sleep(1. / self.RATE - dt)

        rospy.loginfo('>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> Rate: ' + str(1. / dt))

        # cut the part of the path passed
        if self.plan_length == 1:
            pass
        elif closest == self.plan_length - 1:
            # this may happen if we went too far (for example, because of a random lag)
            # leave the last element (otherwise following would be considered finished)
            self.plan = self.plan[closest-1:]
            self.plan_length = self.plan.shape[0]
        else:
            self.plan = self.plan[closest:]
            self.plan_length = self.plan.shape[0]

        self.mutex.release()

    def terminate_following(self, success):
        self.plan_length = 0
        self.plan = np.array([])
        self.set_speed(np.zeros(3))
        if success:
            self.pub_response.publish(self.goal_id + " finished")
        else:
            self.pub_response.publish(self.goal_id + " failed")
        self.goal_id = ''

    def set_plan(self, plan, goal_id):
        self.plan = np.array(plan)
        self.plan_length = self.plan.shape[0]
        self.goal_id = goal_id
        self.t_prev = rospy.get_time()

    def distance(self, x):
        """A function for minimization problem (for finding the closest path point index).
            Distance from path point with index x to robot coords in metric L1."""
        return np.linalg.norm(self.plan[int(x), :2] - self.coords[:2])

    def set_speed(self, vel):
        vx, vy, wz = vel
        tw = Twist()
        tw.linear.x = vx
        tw.linear.y = vy
        tw.angular.z = wz
        self.pub_twist.publish(tw)
        self.vel = vel

    def cmd_callback(self, data):
        self.mutex.acquire()

        # parse name,type
        data_splitted = data.data.split()
        cmd_id = data_splitted[0]
        cmd_type = data_splitted[1]
        cmd_args = data_splitted[2:]

        if cmd_type == "move":  # movement with navigation
            goal_coords = np.array([float(cmd_args[0]), float(cmd_args[1]), float(cmd_args[2])])
            goal = self.coords2pose(goal_coords)

            # check if the goal pose is close to any cube heap
            is_close, heap = self.close_to_heap(goal_coords)
            if is_close:
                success, plan = self.approaching_plan(heap, goal_coords)
                if success:
                    self.set_plan(plan, cmd_id)
            else:
                success, plan = self.request_plan(self.pose, goal)
                if success:
                    self.set_plan(plan, cmd_id)
        elif cmd_type == "move_heap": # approach heap with navigation
            heap_n = int(cmd_args[0])
            angle = float(cmd_args[1])
            goal_coords = np.append(self.heap_coords[heap_n], angle)
            goal_coords[:2] -= self.rotation_transform(np.array([0, 0.060]), goal_coords[2])
            success, plan = self.approaching_plan(self.heap_coords[heap_n], goal_coords)
            if success:
                self.set_plan(plan, cmd_id)
        elif cmd_type == "move_odometry":  # simple liner movement by odometry (if rotation is requested, it will be ignored)
            goal = np.array(cmd_args).astype('float')
            d_map_frame = goal[:2] - self.coords[:2]
            d_robot_frame = self.rotation_transform(d_map_frame, -self.coords[2])
            v = np.abs(d_robot_frame) / np.linalg.norm(d_robot_frame) * self.V_MAX_ODOMETRY_MOVEMENT
            cmd = cmd_id + " 162 " + str(d_robot_frame[0]) + ' ' + str(d_robot_frame[1]) + ' 0 ' + str(v[0]) + ' ' + str(v[1]) + ' 0'
            self.pub_cmd.publish(cmd)
        elif cmd_type == "stop":
            if self.plan_length == 0:
                self.set_speed(np.zeros(3))
            else:
                self.terminate_following(False)

        self.mutex.release()

    def rotation_transform(self, vec, angle):
        M = np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])
        ans = vec.copy()
        ans[:2] = np.matmul(M, ans[:2].reshape((2, 1))).reshape(2)
        return ans

    def approaching_plan(self, heap, goal_coords):
        # distance to the heap
        dist = heap - goal_coords[:2]

        # find the direction e of the approaching movement
        if np.sum(dist ** 2) ** .5 < 1e-4:
            e = np.array([-np.sin(goal_coords[2]), np.cos(goal_coords[2])])
        else:
            e = dist / np.sum(dist ** 2) ** .5

        # via-point from which to approach cubes
        via_point = np.zeros(3)
        via_point[:2] = goal_coords[:2] - e * self.HEAP_APPROACHING_DISTANCE
        via_point[2] = goal_coords[2]  # TODO: make robot face the center of the heap (for safety) regardless of the input

        # get a plan to via-point
        success, plan = self.request_plan(self.pose, self.coords2pose(via_point))
        if success:
            # extend the plan to the goal
            length = np.linalg.norm(goal_coords[:2] - via_point[:2])
            n_extra_points = int(length / self.RESOLUTION)
            extra_path = np.zeros((n_extra_points, 3))
            extra_path[:, 0] = np.linspace(via_point[0], goal_coords[0], num=n_extra_points)
            extra_path[:, 1] = np.linspace(via_point[1], goal_coords[1], num=n_extra_points)
            extra_path[:, 2] = np.ones(n_extra_points) * goal_coords[2]
            return True, np.concatenate((plan, extra_path), axis=0)
        else:
            rospy.loginfo("Follower failed to request an approaching plan.")
            return False, []

    def coords2pose(self, coord):
        position = Point(coord[0], coord[1], 0)
        orientation = Quaternion(*quaternion_from_euler(0, 0, coord[2]))
        return Pose(position, orientation)

    def pose2coords(self, pose):
        angle = euler_from_quaternion([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])[2]
        return np.array([pose.position.x, pose.position.y, angle])

    def request_plan(self, start, finish):
        """ Request a plan from the start Pose (a) to the goal Pose (b). """
        a = PoseStamped(pose=start)
        a.header.frame_id = 'map'

        b = PoseStamped(pose=finish)
        b.header.frame_id = 'map'

        rospy.wait_for_service('move_base/make_plan')
        try:
            make_plan = rospy.ServiceProxy('move_base/make_plan', GetPlan)
            plan = make_plan(a, b, self.GLOBAL_PLAN_TOLERANCE).plan
            path = np.array([[pose.pose.position.x, pose.pose.position.y, euler_from_quaternion([0, 0, pose.pose.orientation.z, pose.pose.orientation.w])[2]] for pose in plan.poses])
            return True, path
        except rospy.ServiceException, e:
            rospy.loginfo("Follower failed to request a plan. Exception: " + str(e))
            self.fail()
            return False, []

    def fail(self):
        return

    def close_to_heap(self, coords):
        dist = np.linalg.norm(self.heap_coords - coords[:2], axis=1)
        if np.min(dist) < self.HEAP_APPROACHING_DISTANCE:
            # return coords of the closest heap if the distance is critical
            return True, self.heap_coords[np.argmin(dist)]
        return False, []


if __name__ == "__main__":
    planner = LocalPlanner()
    rospy.spin()
