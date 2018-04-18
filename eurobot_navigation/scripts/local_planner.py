#!/usr/bin/env python
import rospy
import numpy as np
from tf import TransformListener, LookupException, ConnectivityException, ExtrapolationException
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from scipy.optimize import fminbound
from geometry_msgs.msg import Twist, PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import String, Float32
from nav_msgs.srv import GetPlan, GetMap
from threading import Lock
#from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud

class LocalPlanner:

    # parameters not specific to the robot
    # minimum speed (when approaching the goal and decelerating)
    V_MIN = 0.01
    # plan resolution
    RESOLUTION = 0.005
    # distance for placing a carrot
    D = 0.1
    # on which distance to consider that robot lost it's path
    FOLLOW_TOLERANCE = 0.15
    # goal tolerance
    XY_GOAL_TOLERANCE = 0.005
    YAW_GOAL_TOLERANCE = 0.05
    # slow and fast modes
    mode = 'slow'
    XY_GOAL_TOLERANCE_FAST = 0.015
    YAW_GOAL_TOLERANCE_FAST = 0.1
    # goal tolerance for global planner (when goal of requested plan is obstructed)
    GLOBAL_PLAN_TOLERANCE = 0
    # number of cube heaps
    N_HEAPS = 6
    # max rate of the planner
    RATE = 25
    # max rate of replanning
    PLAN_RATE = 15
    # maximum length of plan when replanning should stop
    REPLANNING_STOP_PLAN_LENGTH = 10
    REPLANNING_STOP_PLAN_LENGTH_HEAP_APPROACH = 50  # empirically determined
    REPLANNING_STOP_PLAN_LENGTH_TOWERS = 100
    # distance between a via-point and a cube heap when approaching the heap
    HEAP_APPROACHING_DISTANCE = 0.17
    # speed for odometry movements
    V_MAX_ODOMETRY_MOVEMENT = 0.4
    # loginfo flag
    LOGINFO = False
    # whether to request a global plan only ones
    ONESHOT = False
    # coefficient for speed limit to avoid collisions
    COLLISION_AVOIDANCE_COEFFICIENT = .33

    def __init__(self):
        rospy.init_node("path_follower", anonymous=True)
        self.robot_name = rospy.get_param("robot_name")
        self.another_robot_name = "main_robot" if self.robot_name == "secondary_robot" else "secondary_robot"
        # robot-specific params
        if self.robot_name == "main_robot":
            # maximum linear and rotational speed
            self.V_MAX = 0.35
            self.W_MAX = 1
            # acceleration of the robot until V_MAX
            self.ACCELERATION = 1
            # length of acceleration/deceleration tracks
            self.D_DECELERATION = 0.15
            self.D_DECELERATION_FAST = 0.05
        else: # if robot_name == "secondary_robot"
            # maximum linear and rotational speed
            self.V_MAX = 0.5
            self.W_MAX = 2.7
            # acceleration of the robot until V_MAX
            self.ACCELERATION = 1
            # length of acceleration/deceleration tracks
            self.D_DECELERATION = 0.7
            self.D_DECELERATION_FAST = 0.1

        # a Lock is used to prevent mixing bytes of diff commands to STM
        self.mutex = Lock()
        
        self.opponent_robots = np.array([])

        # ROS entities
        rospy.init_node("path_follower", anonymous=True)

        rospy.Subscriber("move_command", String, self.cmd_callback, queue_size=1)
        #rospy.Subscriber("odom", Odometry, self.odom_callback, queue_size=1)
        rospy.Subscriber("/map_server/opponent_robots", PointCloud, self.detected_robots_callback, queue_size=1)
        self.pub_twist = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        self.pub_response = rospy.Publisher("response", String, queue_size=10)
        self.pub_cmd = rospy.Publisher("stm_command", String, queue_size=1)
        self.listener = TransformListener()

        # get ROS params
        self.map_service_name = '/' + self.robot_name + '/static_map'
        self.team_color = rospy.get_param("/field/color")
        self.coords = np.array(rospy.get_param('start_' + self.team_color))
        self.coords[:2] /= 1000.0
        self.another_robot_coords = np.array(rospy.get_param("/" + self.another_robot_name + "/start_" + self.team_color))
        self.another_robot_coords[:2] /= 1000.0
        self.pose = self.coords2pose(self.coords)
        self.vel = np.zeros(3)
        # TODO: extrapolate pose (because its update rate is low)

        # get initial cube heap coordinates
        self.heap_coords = np.zeros((self.N_HEAPS, 2))
        for n in range(self.N_HEAPS):
            self.heap_coords[n, 0] = rospy.get_param("/field/cube" + str(n + 1) + "c_x") / 1000
            self.heap_coords[n, 1] = rospy.get_param("/field/cube" + str(n + 1) + "c_y") / 1000

        # get initial water tower coordinates
        self.towers = np.array(rospy.get_param("/field/towers"))
        self.towers[:,:2] /= 1000.0
        self.tower_approaching_vectors = np.array(rospy.get_param("/field/tower_approaching_vectors"))
        self.tower_approaching_vectors[:,:2] /= 1000.0

        self.plan = np.array([])
        self.plan_length = self.plan.shape[0]
        self.goal_id = ''
        self.t_prev = rospy.get_time()
        self.D_steps = int(self.D / self.RESOLUTION)

        # None indicates that replanning timer is off
        self.move_timer = None

        # start the main timer that will follow given paths
        rospy.Timer(rospy.Duration(1.0 / self.RATE), self.next)

    def next(self, event):
        self.mutex.acquire()
        
        try:
            (trans, rot) = self.listener.lookupTransform('/map', '/' + self.robot_name, rospy.Time(0))
            self.pose = Pose(Point(*trans), Quaternion(*rot))
            self.coords = self.pose2coords(self.pose)
            (trans, rot) = self.listener.lookupTransform('/map', '/' + self.robot_name, rospy.Time(0))
            self.another_robot_coords = self.pose2coords(Pose(Point(*trans), Quaternion(*rot)))
        except (LookupException, ConnectivityException, ExtrapolationException):
            # rospy.loginfo("LocalPlanner failed to lookup tf.")
            self.mutex.release()
            return

        if self.plan_length == 0:
            self.mutex.release()
            return

        # FOLLOW THE PLAN
        if self.LOGINFO:
            rospy.loginfo('STARTED NEW ITERATION')
            rospy.loginfo("self coords: " + str(self.coords))
        # current linear and angular goal distance
        goal_distance = self.distance(self.plan_length - 1)
        goal_yaw_distance = min(abs(self.plan[-1][2] - self.coords[2]), 2 * np.pi - abs(self.plan[-1][2] - self.coords[2]))
        goal_yaw_distance = goal_yaw_distance if goal_yaw_distance < np.pi else np.abs(2 * np.pi - goal_yaw_distance)
        if self.LOGINFO:
            rospy.loginfo('goal_distance: ' + str(goal_distance) + ' ; ' + str(goal_yaw_distance))

        # find index of the closest path point by solving an optimization problem
        closest = int(fminbound(self.distance, 0., self.plan_length - 0.1))
        path_deviation = self.distance(closest)
        if self.LOGINFO:
            rospy.loginfo('closest: ' + str(closest))
            rospy.loginfo("closest coords: " + str(self.plan[closest]))
            rospy.loginfo('path_deviation: ' + str(path_deviation))

        # stop and publish response if we have reached the goal with the given tolerance
        if (self.mode == 'slow' and goal_distance < self.XY_GOAL_TOLERANCE and goal_yaw_distance < self.YAW_GOAL_TOLERANCE) or (self.mode == 'fast' and goal_distance < self.XY_GOAL_TOLERANCE_FAST and goal_yaw_distance < self.YAW_GOAL_TOLERANCE_FAST):
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
        if self.LOGINFO:
            rospy.loginfo('carrot = ' + str(carrot))

        # VELOCITY REGULATION.
        # distance to the carrot
        carrot_distance = self.plan[carrot] - self.coords
        carrot_distance[2] = (carrot_distance[2] + np.pi) % (2 * np.pi) - np.pi
        if self.LOGINFO:
            rospy.loginfo('carrot_distance:\t' + str(carrot_distance))
            rospy.loginfo("carrot coords:\t" + str(self.plan[carrot]))

        t0 = rospy.get_time()
        dt = t0 - self.t_prev
        self.t_prev = t0

        # set speed limit
        speed_limit_dec = max(self.V_MIN, goal_distance / self.D_DECELERATION * self.V_MAX)
        if self.LOGINFO:
            rospy.loginfo('speed_limit = ' + str(speed_limit_dec) + ' (deceleration)')
        speed_limit_acs = min(self.V_MAX, np.linalg.norm(self.vel[:2]) + self.ACCELERATION * dt)
        if self.LOGINFO:
            rospy.loginfo('speed_limit = ' + str(speed_limit_acs) + ' (acceleration)')
        speed_limit_collision_avoidance = self.distance_to_closest_robot() * self.COLLISION_AVOIDANCE_COEFFICIENT
        if self.LOGINFO:
            rospy.loginfo('speed_limit = ' + str(speed_limit_collision_avoidance) + ' (collision_avoidance)')

        speed_limit = min(speed_limit_dec, speed_limit_acs, speed_limit_collision_avoidance)
        if self.LOGINFO:
            rospy.loginfo('speed_limit = ' + str(speed_limit))
        # maximum possible speed in carrot distance proportion
        vel = self.V_MAX * carrot_distance / np.max(np.abs(carrot_distance[:2]))
        if abs(vel[2]) > self.W_MAX:
            vel *= self.W_MAX / abs(vel[2])
        if self.LOGINFO:
            rospy.loginfo('vel max\t\t:' + str(vel))

        # apply speed limit
        if np.any(abs(vel[:2]) > speed_limit):
            vel *= speed_limit / np.max(abs(vel[:2]) )
        if self.LOGINFO:
            rospy.loginfo('vel after speed limit\t:' + str(vel))

        # vel in robot frame
        vel_robot_frame = self.rotation_transform(vel, -self.coords[2])
        if self.LOGINFO:
            rospy.loginfo('vel cmd\t:' + str(vel_robot_frame))

        # send speed cmd to the robot
        self.set_speed(vel_robot_frame)

        # sleep to keep the rate
        if dt < 1. / self.RATE:
            rospy.sleep(1. / self.RATE - dt)

        # cut the part of the path passed
        self.plan = self.plan[closest:]
        self.plan_length = self.plan.shape[0]
        if self.LOGINFO:
            rospy.loginfo("new plan length " + str(self.plan_length))
            rospy.loginfo('Rate: ' + str(1. / dt))
        self.mutex.release()

    def terminate_following(self, success, pub_failed=True):
        if self.move_timer is not None:
            self.move_timer.shutdown()
            self.move_timer = None
        self.plan_length = 0
        self.plan = np.array([])
        self.set_speed(np.zeros(3))
        if success:
            self.pub_response.publish(self.goal_id + " finished")
        else:
            if pub_failed:
                self.pub_response.publish(self.goal_id + " failed")
        self.goal_id = ''

    def set_plan(self, plan, goal_id, mode='slow'):
        self.mutex.acquire()

        if self.LOGINFO:
            rospy.loginfo("Setting a new global plan.")
        self.mode = mode
        self.plan = np.array(plan)
        self.plan_length = self.plan.shape[0]
        self.goal_id = goal_id
        self.t_prev = rospy.get_time()

        self.mutex.release()

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

            self.set_move_timer(goal_coords, goal, cmd_id)

        elif cmd_type == "move_heap":  # approach heap with navigation
            heap_n = int(cmd_args[0])
            angle = float(cmd_args[1])
            goal_coords = np.append(self.heap_coords[heap_n], angle)
            goal_coords[:2] -= self.rotation_transform(np.array([0, 0.060]), goal_coords[2])
            goal = self.coords2pose(goal_coords)

            self.set_move_timer(goal_coords, goal, cmd_id)

        elif cmd_type == "move_tower":  # approach water tower with navigation
            tower_n = int(cmd_args[0])

            self.set_move_timer_towers(tower_n, cmd_id)

        elif cmd_type == "move_odometry":  # simple liner movement by odometry (if rotation is requested, it will be ignored)
            goal = np.array(cmd_args).astype('float')
            d_map_frame = goal[:2] - self.coords[:2]
            d_robot_frame = self.rotation_transform(d_map_frame, -self.coords[2])
            v = np.abs(d_robot_frame) / np.linalg.norm(d_robot_frame) * self.V_MAX_ODOMETRY_MOVEMENT
            cmd = cmd_id + " 162 " + str(d_robot_frame[0]) + ' ' + str(d_robot_frame[1]) + ' 0 ' + str(
                v[0]) + ' ' + str(v[1]) + ' 0'
            self.pub_cmd.publish(cmd)

        elif cmd_type == "stop":
            if self.plan_length == 0:
                self.set_speed(np.zeros(3))
            else:
                self.set_speed(np.zeros(3))
                self.terminate_following(False, False)

        self.mutex.release()

    def set_move_timer(self, goal_coords, goal, cmd_id):
        if self.move_timer is not None:
            self.move_timer.shutdown()
            self.move_timer = None
        self.move_timer = rospy.Timer(rospy.Duration(1. / self.PLAN_RATE),
                                      self.get_move_timer_callback(goal_coords, goal, cmd_id), oneshot=self.ONESHOT)

    def get_move_timer_callback(self, goal_coords, goal, cmd_id):
        rospy.loginfo("Creating a timer for replanning.")

        def move_timer(event):
            stop_length = self.REPLANNING_STOP_PLAN_LENGTH
            turn_off_collision_avoidance = False
            if self.robot_name == "main_robot":
                # check if the goal pose is close to any cube heap
                is_close, heap = self.close_to_heap(goal_coords)
                if is_close:
                    stop_length = self.REPLANNING_STOP_PLAN_LENGTH_HEAP_APPROACH
                    turn_off_collision_avoidance = True
                    success, plan = self.approaching_plan(heap, goal_coords)
                    if success:
                        self.set_plan(plan, cmd_id)
                else:
                    success, plan = self.request_plan(self.pose, goal)
                    if success:
                        self.set_plan(plan, cmd_id, 'fast')
            else:
                success, plan = self.request_plan(self.pose, goal)
                if success:
                    self.set_plan(plan, cmd_id, 'fast')
            if self.plan_length <= stop_length:
                if turn_off_collision_avoidance:
                    self.pub_cmd.publish("collision_off 224 0")
                self.move_timer.shutdown()
                self.move_timer = None
                rospy.loginfo("Replanning stopped at plan length = " + str(self.plan_length))
                return

        return move_timer

    def set_move_timer_towers(self, n, cmd_id):
        if self.move_timer is not None:
            self.move_timer.shutdown()
            self.move_timer = None
        self.move_timer = rospy.Timer(rospy.Duration(1. / self.PLAN_RATE),
                                      self.get_move_timer_towers_callback(n, cmd_id))

    def get_move_timer_towers_callback(self, n, cmd_id):
        rospy.loginfo("Creating a timer for replanning (approaching towers).")

        def move_timer(event):
            via_point = self.towers[n] - self.tower_approaching_vectors[n]

            success, plan = self.request_plan(self.pose, self.coords2pose(via_point))

            if success:
                # add bezier path
                P2 = self.towers[n]
                P1 = self.towers[n] - self.tower_approaching_vectors[n]
                len1 = min(int(np.linalg.norm(self.tower_approaching_vectors[n]) / self.RESOLUTION * 2), plan.shape[0])
                P0 = plan[-len1]
                P0[2] = P1[2]
                plan = plan[:-len1]

                if plan.shape[0] != 0:
                    plan[:,2] = self.angle_interpolation(plan[0,2], P1[2], plan.shape[0])
                self.set_plan(np.concatenate((plan, self.bezier_path(P0, P1, P2)), axis=0), cmd_id)

                # length = np.linalg.norm(self.tower_approaching_vectors[n, :2])
                # n_extra_points = int(length / self.RESOLUTION)
                # extra_path = np.zeros((n_extra_points, 3))
                # extra_path[:, 0] = np.linspace(via_point[0], self.towers[n, 0], num=n_extra_points)
                # extra_path[:, 1] = np.linspace(via_point[1], self.towers[n, 1], num=n_extra_points)
                # extra_path[:, 2] = np.ones(n_extra_points) * self.towers[n, 2]
                # self.set_plan(np.concatenate((plan, extra_path), axis=0), cmd_id)

            if self.plan_length <= self.REPLANNING_STOP_PLAN_LENGTH_TOWERS:
                self.move_timer.shutdown()
                self.move_timer = None
                rospy.loginfo("Replanning stopped at plan length = " + str(self.plan_length))
                return

        return move_timer

    def rotation_transform(self, vec, angle):
        M = np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])
        ans = vec.copy()
        ans[:2] = np.matmul(M, ans[:2].reshape((2, 1))).reshape(2)
        return ans

    def approaching_plan(self, heap, goal_coords):
        # distance to the    heap
        dist = heap - goal_coords[:2]

        # find the direction e of the approaching movement
        if np.sum(dist ** 2) ** .5 < 1e-4:
            e = np.array([-np.sin(goal_coords[2]), np.cos(goal_coords[2])])
        else:
            e = dist / np.sum(dist ** 2) ** .5

        # via-point from which to approach cubes
        via_point = np.zeros(3)
        via_point[:2] = goal_coords[:2] - e * self.HEAP_APPROACHING_DISTANCE
        via_point[2] = goal_coords[
            2]  # TODO: make robot face the center of the heap (for safety) regardless of the input

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
        angle = euler_from_quaternion([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])[
            2]
        return np.array([pose.position.x, pose.position.y, angle])

    def request_plan(self, start, finish):
        """ Request a plan from the start Pose (a) to the goal Pose (b). """
        a = PoseStamped(pose=start)
        a.header.frame_id = 'map'

        b = PoseStamped(pose=finish)
        b.header.frame_id = 'map'

        success, occupied = self.are_occupied([a, b])
        if not success:
            return False, []
        if occupied.any():
            if occupied[1]:
                rospy.loginfo("Follower failed to request a plan. Goal cell on the map is occupied!")
            elif occupied[0]:
                rospy.loginfo("Follower failed to request a plan. Starting point on the map is occupied!")
            self.fail()
            return False, []

        rospy.wait_for_service('global_planner/planner/make_plan')
        try:
            make_plan = rospy.ServiceProxy('global_planner/planner/make_plan', GetPlan)
            plan = make_plan(a, b, self.GLOBAL_PLAN_TOLERANCE).plan
            path = np.array([[pose.pose.position.x, pose.pose.position.y,
                              euler_from_quaternion([0, 0, pose.pose.orientation.z, pose.pose.orientation.w])[2]] for
                             pose in plan.poses])
            return True, path
        except rospy.ServiceException, e:
            rospy.loginfo("Follower failed to request a plan. Exception: " + str(e))
            self.fail()
            return False, []

    def are_occupied(self, points):
        rospy.wait_for_service(self.map_service_name)
        try:
            get_map = rospy.ServiceProxy(self.map_service_name, GetMap)
            costmap = get_map().map
            res = costmap.info.resolution
            print points
            return True, np.array([costmap.data[costmap.info.width * int(point.pose.position.y / res + 2) + int(
                point.pose.position.x / res + 2)] == 100 for point in points])
        except Exception, e:
            rospy.loginfo("Failed to use /static_map service. Exception: " + str(e))
            return False, []

    def fail(self):
        return

    def close_to_heap(self, coords):
        dist = np.linalg.norm(self.heap_coords - coords[:2], axis=1)
        if np.min(dist) < self.HEAP_APPROACHING_DISTANCE:
            # return coords of the closest heap if the distance is critical
            return True, self.heap_coords[np.argmin(dist)]
        return False, []

    def bezier_curve(self, P0, P1, P2, t):
        return (1 - t) ** 2 * P0[:2] + 2 * t * (1 - t) * P1[:2] + t ** 2 * P2[:2]

    def bezier_path(self, P0, P1, P2):
        max_len = int(((np.linalg.norm(P1 - P0) + np.linalg.norm(P2 - P1))) / self.RESOLUTION)
        path = np.zeros((max_len, 3))
        t = 0
        dt = 1.0 / (5 * max_len)

        path[0, :2] = P0[:2]
        i = 1

        while t < 1:
            t += dt
            potential_point = self.bezier_curve(P0, P1, P2, t)
            if np.linalg.norm(path[i - 1, :2] - potential_point) >= self.RESOLUTION:
                path[i, :2] = potential_point
                i += 1
                if np.linalg.norm(P2[:2] - potential_point) < self.RESOLUTION:
                    break
        path[i, :2] = P2[:2]
        n = i + 1
        path[:n, 2] = np.linspace(P0[2], P2[2], n)
        return path[:n]

    def angle_interpolation(sel, a, b, n):
        a = a % (2 * np.pi)
        b = b % (2 * np.pi)
        if a - b > np.pi:
            b += 2 * np.pi
        elif b - a > np.pi:
            a += 2 * np.pi
        return np.linspace(a, b, n) % (2 * np.pi)

    #def odom_callback(self, odom):
        #self.vel = np.array([odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.angular.z])

    def detected_robots_callback(self, data):
        if len(data.points) == 0:
            self.opponent_robots = np.array([])
        else:
            self.opponent_robots = np.array([[robot.x, robot.y] for robot in data.points])
        self.robots_upd_time = data.header.stamp

    def distance_to_closest_robot(self):
        ans = np.linalg.norm(self.another_robot_coords)
        if self.opponent_robots.shape[0] > 0:
            ans = min(ans, np.min(np.linalg.norm(self.opponent_robots)))
        return ans

if __name__ == "__main__":
    planner = LocalPlanner()
    rospy.spin()
