#!/usr/bin/env python
import rospy
import numpy as np
from tf import TransformListener, LookupException, ConnectivityException, ExtrapolationException
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from threading import Lock


class MotionPlanner:
    def __init__(self):
        rospy.init_node("motion_planner", anonymous=True)

        self.robot_name = rospy.get_param("robot_name")
        self.team_color = rospy.get_param("/field/color")
        self.coords = np.array(rospy.get_param('start_' + self.team_color))
        self.coords[:2] /= 1000.0
        self.vel = np.zeros(3)

        self.RATE = rospy.get_param("motion_planner/RATE")
        self.XY_GOAL_TOLERANCE = rospy.get_param("motion_planner/XY_GOAL_TOLERANCE")
        self.YAW_GOAL_TOLERANCE = rospy.get_param("motion_planner/YAW_GOAL_TOLERANCE")
        self.V_MAX = rospy.get_param("motion_planner/V_MAX")
        self.W_MAX = rospy.get_param("motion_planner/W_MAX")
        self.ACCELERATION = rospy.get_param("motion_planner/ACCELERATION")
        self.D_DECELERATION = rospy.get_param("motion_planner/D_DECELERATION")
        # for movements to water towers or cube heaps
        self.XY_ACCURATE_GOAL_TOLERANCE = rospy.get_param("motion_planner/XY_ACCURATE_GOAL_TOLERANCE")
        self.YAW_ACCURATE_GOAL_TOLERANCE = rospy.get_param("motion_planner/YAW_ACCURATE_GOAL_TOLERANCE")
        self.D_ACCURATE_DECELERATION = rospy.get_param("motion_planner/D_ACCURATE_DECELERATION")

        if self.robot_name == "main_robot":
            # get initial cube heap coordinates
            self.heap_coords = np.zeros((6, 2))
            for n in range(6):
                self.heap_coords[n, 0] = rospy.get_param("/field/cube" + str(n + 1) + "c_x") / 1000
                self.heap_coords[n, 1] = rospy.get_param("/field/cube" + str(n + 1) + "c_y") / 1000
        else:
            # get water tower coordinates
            self.towers = np.array(rospy.get_param("/field/towers"))
            self.towers[:, :2] /= 1000.0
            self.tower_approaching_vectors = np.array(rospy.get_param("/field/tower_approaching_vectors"))
            self.tower_approaching_vectors[:, :2] /= 1000.0

        self.mutex = Lock()

        self.cmd_id = None
        self.t_prev = None
        self.goal = None
        self.mode = None

        self.pub_twist = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        self.pub_response = rospy.Publisher("response", String, queue_size=10)
        self.pub_cmd = rospy.Publisher("stm_command", String, queue_size=1)
        self.listener = TransformListener()
        rospy.Subscriber("move_command", String, self.cmd_callback, queue_size=1)

        # start the main timer that will follow given goal points
        rospy.Timer(rospy.Duration(1.0 / self.RATE), self.plan)

    def plan(self, event):
        self.mutex.acquire()

        if self.cmd_id is None:
            self.mutex.release()
            return

        rospy.loginfo("-------NEW MOTION PLANNING ITERATION-------")

        try:
            (trans, rot) = self.listener.lookupTransform('/map', '/' + self.robot_name, rospy.Time(0))
            angle = euler_from_quaternion(rot)[2]
            self.coords = np.array([trans[0], trans[1], angle])
            rospy.loginfo("Robot coords:\t" + str(self.coords))
            rospy.loginfo("Goal coords:\t" + str(self.goal))
        except (LookupException, ConnectivityException, ExtrapolationException):
            rospy.loginfo("MotionPlanner failed to lookup tf.")
            self.mutex.release()
            return

        # current linear and angular goal distance
        goal_distance = np.zeros(3)
        goal_distance[:2] = self.goal[:2] - self.coords[:2]
        goal_distance[2] = min(abs(self.goal[2] - self.coords[2]), 2 * np.pi - abs(self.goal[2] - self.coords[2]))
        rospy.loginfo('Goal distance:\t' + str(goal_distance))
        goal_d = np.linalg.norm(goal_distance[:2])
        rospy.loginfo('Goal d:\t' + str(goal_d))

        # stop and publish response if we have reached the goal with the given tolerance
        if (self.mode == 'normal' and goal_d < self.XY_GOAL_TOLERANCE and goal_distance[2] < self.YAW_GOAL_TOLERANCE) or (self.mode == 'accurate' and goal_d < self.XY_ACCURATE_GOAL_TOLERANCE and goal_distance[2] < self.YAW_ACCURATE_GOAL_TOLERANCE):
            rospy.loginfo(self.cmd_id + " finished, reached the goal")
            self.terminate_following()
            self.mutex.release()
            return

        # CHOOSE VELOCITY COMMAND.

        t = rospy.get_time()
        dt = t - self.t_prev
        self.t_prev = t

        # set speed limit
        speed_limit_acs = min(self.V_MAX, np.linalg.norm(self.vel[:2]) + self.ACCELERATION * dt)
        rospy.loginfo('Acceleration Speed Limit:\t' + str(speed_limit_acs))

        speed_limit_dec = goal_d / self.D_DECELERATION * self.V_MAX
        rospy.loginfo('Deceleration Speed Limit:\t' + str(speed_limit_dec))

        speed_limit = min(speed_limit_dec, speed_limit_acs)
        rospy.loginfo('Final Speed Limit:\t' + str(speed_limit))

        # maximum speed in goal distance proportion
        vel = self.V_MAX * goal_distance / goal_d
        if abs(vel[2]) > self.W_MAX:
            vel *= self.W_MAX / abs(vel[2])
        rospy.loginfo('Vel before speed limit\t:' + str(vel))

        # apply speed limit
        v_abs = np.linalg.norm(vel[:2])
        rospy.loginfo('Vel abs before speed limit\t:' + str(vel))
        if v_abs > speed_limit:
            vel *= speed_limit / v_abs
        rospy.loginfo('Vel after speed limit\t:' + str(vel))

        # send cmd: vel in robot frame
        vel_robot_frame = self.rotation_transform(vel, -self.coords[2])
        self.set_speed(vel_robot_frame)
        rospy.loginfo('Vel cmd\t:' + str(vel_robot_frame))

        self.mutex.release()

    @staticmethod
    def rotation_transform(vec, angle):
        M = np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])
        ans = vec.copy()
        ans[:2] = np.matmul(M, ans[:2].reshape((2, 1))).reshape(2)
        return ans

    def terminate_following(self):
        self.set_speed(np.zeros(3))
        self.pub_response.publish(self.cmd_id + " finished")
        self.cmd_id = None
        self.t_prev = None
        self.goal = None
        self.mode = None

    def set_goal(self, goal, cmd_id, mode='normal'):
        rospy.loginfo("Setting a new goal:\t" + str(goal))
        self.cmd_id = cmd_id
        self.t_prev = rospy.get_time()
        self.goal = goal
        self.mode = mode

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
        rospy.loginfo("NEW CMD:\t" + str(data.data))

        # parse name,type
        data_splitted = data.data.split()
        cmd_id = data_splitted[0]
        cmd_type = data_splitted[1]
        cmd_args = data_splitted[2:]

        if cmd_type == "move":  # movement with navigation
            path = np.array(cmd_args).astype('float').reshape((len(cmd_args) / 3,3))
            self.set_path(path, cmd_id)

        elif cmd_type == "move_heap":  # approach heap with navigation
            heap_n = int(cmd_args[0])
            angle = float(cmd_args[1])
            goal_coords = np.append(self.heap_coords[heap_n], angle)
            goal_coords[:2] -= self.rotation_transform(np.array([0, 0.060]), goal_coords[2])

        elif cmd_type == "move_tower":  # approach water tower with navigation
            tower_n = int(cmd_args[0])
            # TODO

        elif cmd_type == "move_odometry":  # simple liner movement by odometry (if rotation is requested, it will be ignored)
            inp = np.array(cmd_args).astype('float')
            self.move_odometry(cmd_id, inp[:2], inp[2])

        elif cmd_type == "rotate_odometry":  # simple rotation by odometry
            inp = np.array(cmd_args).astype('float')
            self.rotate_odometry(cmd_id, inp[0], inp[1])

        elif cmd_type == "stop":
            self.terminate_following()

        self.mutex.release()

    def move_odometry(self, cmd_id, goal, vel):
        rospy.loginfo("-------NEW LINEAR ODOMETRY MOVEMENT-------")
        rospy.loginfo("Goal:\t" + str(goal))
        try:
            (trans, rot) = self.listener.lookupTransform('/map', '/' + self.robot_name, rospy.Time(0))
            angle = euler_from_quaternion(rot)[2]
            self.coords = np.array([trans[0], trans[1], angle])
            rospy.loginfo("Robot coords:\t" + str(self.coords))
        except (LookupException, ConnectivityException, ExtrapolationException):
            rospy.loginfo("MotionPlanner failed to lookup tf.")
            return

        d_map_frame = goal[:2] - self.coords[:2]
        rospy.loginfo("Distance in map frame:\t" + str(d_map_frame))
        d_robot_frame = self.rotation_transform(d_map_frame, -self.coords[2])
        v = np.abs(d_robot_frame) / np.linalg.norm(d_robot_frame) * vel
        cmd = cmd_id + " 162 " + str(d_robot_frame[0]) + ' ' + str(d_robot_frame[1]) + ' 0 ' + str(
            v[0]) + ' ' + str(v[1]) + ' 0'
        rospy.loginfo("Sending cmd:\t" + cmd)
        self.pub_cmd.publish(cmd)

    def rotate_odometry(self, cmd_id, goal_angle, w):
        rospy.loginfo("-------NEW ROTATIONAL ODOMETRY MOVEMENT-------")
        rospy.loginfo("Goal angle:\t" + str(goal_angle))
        try:
            (trans, rot) = self.listener.lookupTransform('/map', '/' + self.robot_name, rospy.Time(0))
            angle = euler_from_quaternion(rot)[2]
            self.coords = np.array([trans[0], trans[1], angle])
            rospy.loginfo("Robot coords:\t" + str(self.coords))
        except (LookupException, ConnectivityException, ExtrapolationException):
            rospy.loginfo("MotionPlanner failed to lookup tf.")
            return

        delta_angle = goal_angle - self.coords[2]
        rospy.loginfo("Delta angle:\t" + str(delta_angle))
        cmd = cmd_id + " 162 0 0" + str(delta_angle) + ' 0 0 ' + str(w)
        rospy.loginfo("Sending cmd:\t" + cmd)
        self.pub_cmd.publish(cmd)


if __name__ == "__main__":
    planner = MotionPlanner()
    rospy.spin()
