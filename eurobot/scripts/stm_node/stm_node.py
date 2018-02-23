#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from STMprotocol import STMprotocol
from threading import Lock
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import tf
import numpy as np


class stm_node(STMprotocol):
    def __init__(self, serial_port):
        super(stm_node, self).__init__(serial_port)
        self.mutex = Lock()

        # ROS
        rospy.init_node('stm_node', anonymous=True)
        rospy.Subscriber("stm_command", String, self.stm_command_callback)
        rospy.Subscriber("cmd_vel", Twist, self.set_twist)
        self.pub_response = rospy.Publisher("response", String, queue_size=10)
        self.pub_odom = rospy.Publisher("odom", Odometry, queue_size=1)

        self.robot_name = rospy.get_param("robot_name")
        self.br = tf.TransformBroadcaster()

        # high-level commands info (for handling response)
        # TODO: test and debug
        self.actions_in_progress = ['']  # action_names, indexing corresponds to types indexing
        self.action_types = []  # list of high-level action types only

        rospy.Timer(rospy.Duration(1./40), self.pub_timer_callback)


    def set_twist(self, twist):
        self.send("set_speed", 8, [twist.linear.x, twist.linear.y, twist.angular.z])


    def parse_data(self, data):
        data_splitted = data.data.split()
        action_name = data_splitted[0]
        action_type = int(data_splitted[1])
        args_str = data_splitted[2:]
        # TODO: split any chars in Strings like 'ECHO'->['E','C','H','O']
        action_args_dict = {'B': int, 'H': int, 'f': float}
        args = [action_args_dict[t](s) for t, s in zip(self.pack_format[action_type][1:], args_str)]
        return action_name, action_type, args


    def stm_command_callback(self, data):
        action_name, action_type, args = self.parse_data(data)
        self.send(action_name, action_type, args)


    def send(self, action_name, action_type, args):
        # Lock() is used to prevent mixing bytes of diff commands to STM
        self.mutex.acquire()
        # send command to STM32
        successfully, args_response = self.send_command(action_type, args)
        self.mutex.release()

        # high-level commands handling
        if action_type in self.action_types:
            # store action_name
            self.actions_in_progress[self.action_types[action_type]] = action_name

        return successfully, args_response


    def handle_response(self, status):  # TODO
        """Handles response for high-lvl commands (only)."""
        l = len(status)
        for i in range(l):
            # mind that indeces in status[] correspond to indexes in actions_in_progress[]
            if status[i] == '0' and len(self.actions_in_progress[i]) > 0:
                self.actions_in_progress[i] = ''  # stop storing this action_name
                self.pub_response.publish(self.actions_in_progress[i] + " done")  # publish response


    def publish_odom(self, coords, vel):
        odom = Odometry()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = self.robot_name
        odom.pose.pose.position.x = coords[0]
        odom.pose.pose.position.y = coords[1]
        quat = tf.transformations.quaternion_from_euler(0, 0, coords[2])
        odom.pose.pose.orientation.z = quat[2]
        odom.pose.pose.orientation.w = quat[3]
        odom.twist.twist.linear.x = vel[0]
        odom.twist.twist.linear.x = vel[1]
        odom.twist.twist.angular.z = vel[2]
        self.pub_odom.publish(odom)

        self.br.sendTransform((coords[0], coords[1], 0),
                                    tf.transformations.quaternion_from_euler(0, 0, coords[2]),
                                    rospy.Time.now(),
                                    self.robot_name,
                                    "odom")

        self.br.sendTransform((0, 0.06, 0.41),
                                    tf.transformations.quaternion_from_euler(0, 0, 1.570796),
                                    rospy.Time.now(),
                                    'laser',
                                    self.robot_name)

    def pub_timer_callback(self, event):
        successfully1, coords = stm.send('request_stm_coords', 15, [])
        successfully2, vel = stm.send('request_stm_vel', 9, [])
        if successfully1 and successfully2:
            stm.publish_odom(coords, vel)
        # TODO: status = ...
        # TODO: stm.handle_response(status)


if __name__ == '__main__':
    serial_port = "/dev/ttyUSB0"
    stm = stm_node(serial_port)

    # turn stm inverse kinematics handler ON
    stm.send("set_inverse_kinematics_ON", 13, [1])

    # set initial coords in STM
    initial_coords = [rospy.get_param('start_x') / 1000.0, rospy.get_param('start_y') / 1000.0, rospy.get_param('start_a')];
    stm.send("set_initial_coords", 14, initial_coords)

    rospy.spin()
