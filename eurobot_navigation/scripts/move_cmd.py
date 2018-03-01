#!/usr/bin/env python
import numpy as np
import rospy
from std_msgs.msg import String
#import tf
from move_base_msgs.msg import MoveBaseActionGoal
from actionlib_msgs.msg import GoalStatusArray


class MovementHandler():
    def __init__(self):
        rospy.init_node('movement_handler', anonymous=True)
        rospy.Subscriber("move_command", String, self.cmd_callback)
        self.coords = np.zeros(3)
        coords_source = rospy.get_param("track_regulator/coords_source")
        rospy.Subscriber("%s/coordinates" % coords_source, String, self.coordinates_callback)
        self.pub_response = rospy.Publisher("response", String, queue_size=10)
        self.pub_goal = rospy.Publisher("/move_base/goal", MoveBaseActionGoal, queue_size=1)
        rospy.Subscriber("/move_base/status",)
        self.d = 0.1 # first approach distance

    @staticmethod
    def parse_data(data):
        data_splitted = data.data.split()
        cmd_id = data_splitted[0]
        cmd = int(data_splitted[1])
        args_str = data_splitted[2:]
        return cmd_id, cmd, args_str

    def perform_cmd(self, cmd_id, cmd, args_str):
        if cmd == "MOVE":
            # parse args
            args = [float(args_str[i]) for i in range(3)]
            # start movement
            self.move(*args)
        elif cmd == "STOP":
            pass
            # pub_stop
        elif cmd == "RETURN":
            x = rospy.get_param("/main_robot/start_x")
            y = rospy.get_param("/main_robot/start_y")
            a = rospy.get_param("/main_robot/start_a")
            self.move(x, y, a)
        elif cmd == "MOVECUBE":
            n = int(args_str[0])
            self.move_to_heap(n, 0)
            
    def move(self, x, y, a):
        goal = MoveBaseActionGoal()
        goal.goal.target_pose.header.frame_id = 'odom'
        goal.goal.target_pose.pose.position.x = x
        goal.goal.target_pose.pose.position.y = y
        orient = tf.transformations.quaternion_from_euler(0, 0, a)
        goal.goal.target_pose.pose.orientation.z = orient[2]
        goal.goal.target_pose.pose.orientation.w = orient[3]
        self.pub_goal.publish(goal)

    def move_to_heap(self, n, a): # should be checked
        # calculate approach coords
        x_center = rospy.get_param("/field/cube" + str(n_cube) + "c_x") / 1000
        y_center = rospy.get_param("/field/cube" + str(n_cube) + "c_y") / 1000
        coords = self.coords
        dx = x_center - coords[0]
        dy = y_center - coords[1]
        a_approach = np.arctan2(dy, dx)
        x_d = x_center - d * np.cos(a_approach)
        y_d = y_center - d * np.sin(a_approach)
        x_approach = x_center - x_d
        y_approach = y_center - y_d 
        self.move(x_approach, y_approach, a_approach)
        self.move_linearly(x_d, y_d, 0)
        self.move_linearly(0, 0, a)

    def move_linearly(self, x, y, a): #accurate movements
        return

    def cmd_callback(self, data):
        cmd_id, cmd, args_str = self.parse_data(data)
        status = self.perform_cmd(cmd_id, cmd, args_str)
        self.pub_response()

    @staticmethod
    def coordinates_callback(data):
        data_splitted = data.data.split()
        global coords
        coords = np.array([float(data_splitted[i]) for i in range(3)])


if __name__ == '__main__':

    rospy.spin()
