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
        self.heap_n = None

        self.cmd_stop_robot_id = None
        self.stop_id = 0

        self.pub_twist = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        self.pub_response = rospy.Publisher("response", String, queue_size=10)
        self.pub_cmd = rospy.Publisher("stm_command", String, queue_size=1)
        self.listener = TransformListener()
        rospy.Subscriber("move_command", String, self.cmd_callback, queue_size=1)
        rospy.Subscriber("response", String, self.response_callback, queue_size=1)

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
            angle = euler_from_quaternion(rot)[2] % (2 * np.pi)
            self.coords = np.array([trans[0], trans[1], angle])
            rospy.loginfo("Robot coords:\t" + str(self.coords))
            rospy.loginfo("Goal coords:\t" + str(self.goal))
        except (LookupException, ConnectivityException, ExtrapolationException):
            rospy.loginfo("MotionPlanner failed to lookup tf.")
            self.mutex.release()
            return

        # current linear and angular goal distance
        goal_distance = np.zeros(3)
        goal_distance = self.distance(self.coords, self.goal)
        #goal_distance[2] = min(abs(self.goal[2] - self.coords[2]), 2 * np.pi - abs(self.goal[2] - self.coords[2]))
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
    def distance(coords1, coords2):
        ans = coords2 - coords1
        if abs(coords1[2] - coords2[2]) > np.pi:
            if coords2[2] > coords1[2]:
                ans[2] -= 2 * np.pi
            else:
                ans[2] += 2 * np.pi
        return ans
    
    @staticmethod
    def rotation_transform(vec, angle):
        M = np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])
        ans = vec.copy()
        ans[:2] = np.matmul(M, ans[:2].reshape((2, 1))).reshape(2)
        return ans

    def terminate_following(self):
        rospy.loginfo("Setting robot speed to zero.")
        self.set_speed(np.zeros(3))
        self.stop_robot()
        rospy.loginfo("Robot has stopped. Starting correction by odometry movement.")
        self.move_odometry(self.cmd_id, *self.goal)
        #self.pub_response.publish(self.cmd_id + " finished")
        self.cmd_id = None
        self.t_prev = None
        self.goal = None
        self.mode = None
        self.heap_n = None

    def stop_robot(self):
        self.cmd_stop_robot_id = "stop_" + self.robot_name + str(self.stop_id)
        self.stop_id += 1
        cmd = self.cmd_stop_robot_id + " 8 0 0 0"
        self.robot_stopped = False
        self.pub_cmd.publish(cmd)
        while not self.robot_stopped:
            rospy.sleep(1.0 / 40)
        #rospy.sleep(0.5)
        self.cmd_stop_robot_id = None

    def set_goal(self, goal, cmd_id, mode='normal', heap_n=0):
        rospy.loginfo("Setting a new goal:\t" + str(goal))
        self.cmd_id = cmd_id
        self.t_prev = rospy.get_time()
        self.goal = goal
        self.mode = mode
        if mode == "heap":
            self.heap_n = heap_n

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
        rospy.loginfo("----------------------------------")
        rospy.loginfo("NEW CMD:\t" + str(data.data))

        # parse name,type
        data_splitted = data.data.split()
        cmd_id = data_splitted[0]
        cmd_type = data_splitted[1]
        cmd_args = data_splitted[2:]

        if cmd_type == "move":
            goal = np.array(cmd_args).astype('float') #.reshape((len(cmd_args) / 3,3))
            goal[2] %= (2 * np.pi)
            self.set_goal(goal, cmd_id)

        elif cmd_type == "move_heap":  # approach heap
            n = int(cmd_args[0])
            #final_angle = float(cmd_args[1]) TODO
            self.move_heap(cmd_id, n)

        elif cmd_type == "move_odometry":  # simple movement by odometry
            inp = np.array(cmd_args).astype('float')
            self.move_odometry(cmd_id, *inp)
        
        elif cmd_type == "translate_odometry":  # simple liner movement by odometry
            inp = np.array(cmd_args).astype('float')
            self.translate_odometry(cmd_id, *inp)

        elif cmd_type == "rotate_odometry":  # simple rotation by odometry
            inp = np.array(cmd_args).astype('float')
            self.rotate_odometry(cmd_id, *inp)

        elif cmd_type == "stop":
            self.terminate_following()

        self.mutex.release()

    def move_heap(self, cmd_id, n):
        if not self.update_coords():
            return
        angle = np.arctan2(self.heap_coords[1] - self.coords[1], self.heap_coords[0] - self.coords[0])
        self.rotate_odometry(cmd_id, angle)
        goal_coords = np.append(self.heap_coords[heap_n], angle)
        self.set_goal(goal, cmd_id, mode="accurate") 


    def move_odometry(self, cmd_id, goal_x, goal_y, goal_a, vel=0.3, w=1.5):
        goal = np.array([goal_x, goal_y, goal_a])
        rospy.loginfo("-------NEW ODOMETRY MOVEMENT-------")
        rospy.loginfo("Goal:\t" + str(goal))
        if not self.update_coords():
            return

        d_map_frame = self.distance(self.coords, goal)
        rospy.loginfo("Distance in map frame:\t" + str(d_map_frame))
        d_robot_frame = self.rotation_transform(d_map_frame, -self.coords[2])
        rospy.loginfo("Distance in robot frame:\t" + str(d_robot_frame))
        d = np.linalg.norm(d_robot_frame[:2])
        rospy.loginfo("Distance:\t" + str(d))

        beta = np.arctan2(d_robot_frame[1], d_robot_frame[0])
        rospy.loginfo("beta:\t" + str(beta))
        da = d_robot_frame[2]
        dx = d * np.cos(beta - da / 2)
        dy = d * np.sin(beta - da / 2)
        d_cmd = np.array([dx, dy, da])
        if da != 0:
            d_cmd[:2] *= da / (2 * np.sin(da / 2))
        rospy.loginfo("d_cmd:\t" + str(d_cmd))

        v_cmd = np.abs(d_cmd) / np.linalg.norm(d_cmd[:2]) * vel
        if abs(v_cmd[2]) > w:
            v_cmd *= w / abs(v_cmd[2])
        rospy.loginfo("v_cmd:\t" + str(v_cmd))
        d_cmd[2] = v_cmd[2] = 0
        cmd = cmd_id + " 162 " + str(d_cmd[0]) + " " + str(d_cmd[1]) + " " + str(d_cmd[2]) + " " + str(v_cmd[0]) + " " + str(v_cmd[1]) + " " + str(v_cmd[2])
        rospy.loginfo("Sending cmd: " + cmd)
        self.pub_cmd.publish(cmd)

    def translate_odometry(self, cmd_id, goal_x, goal_y, vel=0.2):
        goal = np.array([goal_x, goal_y])
        rospy.loginfo("-------NEW LINEAR ODOMETRY MOVEMENT-------")
        rospy.loginfo("Goal:\t" + str(goal))
        if not self.update_coords():
            return

        d_map_frame = goal[:2] - self.coords[:2]
        rospy.loginfo("Distance in map frame:\t" + str(d_map_frame))
        d_robot_frame = self.rotation_transform(d_map_frame, -self.coords[2])
        v = np.abs(d_robot_frame) / np.linalg.norm(d_robot_frame) * vel
        cmd = cmd_id + " 162 " + str(d_robot_frame[0]) + ' ' + str(d_robot_frame[1]) + ' 0 ' + str(
            v[0]) + ' ' + str(v[1]) + ' 0'
        rospy.loginfo("Sending cmd: " + cmd)
        self.pub_cmd.publish(cmd)

    def rotate_odometry(self, cmd_id, goal_angle, w=1.0):
        rospy.loginfo("-------NEW ROTATIONAL ODOMETRY MOVEMENT-------")
        rospy.loginfo("Goal angle:\t" + str(goal_angle))
        if not self.update_coords():
            return

        delta_angle = goal_angle - self.coords[2]
        rospy.loginfo("Delta angle:\t" + str(delta_angle))
        cmd = cmd_id + " 162 0 0 " + str(delta_angle) + ' 0 0 ' + str(w)
        rospy.loginfo("Sending cmd: " + cmd)
        self.pub_cmd.publish(cmd)

    def update_coords(self):
        try:
            (trans, rot) = self.listener.lookupTransform('/map', '/' + self.robot_name, rospy.Time(0))
            angle = euler_from_quaternion(rot)[2] % (2 * np.pi)
            self.coords = np.array([trans[0], trans[1], angle])
            rospy.loginfo("Robot coords:\t" + str(self.coords))
            return True
        except (LookupException, ConnectivityException, ExtrapolationException):
            rospy.loginfo("MotionPlanner failed to lookup tf.")
            return False

    def response_callback(self, data):
        if self.cmd_stop_robot_id is None:
            return
        data_splitted = data.data.split()
        if data_splitted[0] == self.cmd_stop_robot_id and data_splitted[1] == "finished":
            self.robot_stopped = True

if __name__ == "__main__":
    planner = MotionPlanner()
    rospy.spin()
