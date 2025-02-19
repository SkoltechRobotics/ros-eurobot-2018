#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from STMprotocol import STMprotocol
from threading import Lock
from geometry_msgs.msg import Twist, TransformStamped
from std_msgs.msg import Int32MultiArray
import tf2_ros
import tf_conversions
import numpy as np

STATUS_RATE = 20
RF_RATE = 20
STM_COORDS_RATE = 40

GET_ODOMETRY_MOVEMENT_STATUS = 0xa0
GET_MANIPULATOR_STATUS = 0xa1
GET_STARTUP_STATUS = 0xa3
GET_SEC_ROBOT_MANIPULATOR_STATUS = 0xc0

MANIPULATOR_JOBS = [0xb0, 0xb1, 0xb2, 0xb3, 0xb4, 0xb5, 0xb7, 0xc1, 0xc2, 0xc3]
IMMEDIATE_FINISHED = [0xc4, 0xb6, 0xe0]
ODOMETRY_MOVEMENT = [0xa2]
DEBUG_COMMANDS = [0x0c]

UNLOAD_TOWER = 0xb1
CMD_VEL = 0x08
REQUEST_RF_DATA = 0xd0
REQUEST_RF_DATA_SECONDARY = 0xd1
BAUD_RATE = {"main_robot": 250000,
             "secondary_robot": 250000}


# noinspection PyTypeChecker,PyUnusedLocal
class StmNode(STMprotocol):
    # noinspection PyTypeChecker
    def __init__(self, serial_port):
        # Init ROS
        rospy.init_node('stm_node', anonymous=True)

        # Get ROS parameters
        self.robot_name = rospy.get_param("robot_name")
        self.color = rospy.get_param("/field/color")

        # ROS publishers
        self.pub_response = rospy.Publisher("response", String, queue_size=10)
        self.pub_rf = rospy.Publisher("barrier_rangefinders_data", Int32MultiArray, queue_size=10)
        self.pub_wire = rospy.Publisher("/server/wire_status", String, queue_size=100)

        # ROS subscribers
        rospy.Subscriber("stm_command", String, self.stm_command_callback)
        rospy.Subscriber("cmd_vel", Twist, self.set_twist)
        rospy.Subscriber("/server/stm_node_command", String, self.stm_node_command_callback)

        # Init STM protocol class
        super(StmNode, self).__init__(serial_port, BAUD_RATE[self.robot_name])
        self.mutex = Lock()

        # turn stm inverse kinematics handler ON
        self.send("set_inverse_kinematics_ON", 13, [1])

        # TF transform broadcaster
        self.br = tf2_ros.TransformBroadcaster()

        # high-level command IDs
        self.odometry_movement_id = ''
        self.timer_odom_move = None

        # storage for timer objects and cmd_names (for 3 manipulators)
        self.timer_m = [None] * 3
        self.take_cube = [''] * 3

        # set initial coords in STM
        self.initial_coords = np.array(rospy.get_param('start_' + self.color))
        self.initial_coords[:2] /= 1000.0
        self.send("set_initial_coords", 14, self.initial_coords)

        # get LIDAR coords
        self.laser_coords = (rospy.get_param('lidar_x') / 1000.0, rospy.get_param('lidar_y') / 1000.0, 0.41)
        self.laser_angle = rospy.get_param('lidar_a')

        # start routine jobs
        rospy.Timer(rospy.Duration(1. / STM_COORDS_RATE), self.pub_odom_coords_timer_callback)
        rospy.Timer(rospy.Duration(1. / RF_RATE), self.pub_rf_data_timer_callback)
        self.wire_timer = rospy.Timer(rospy.Duration(1. / STATUS_RATE), self.wire_timer_callback)

    def set_twist(self, twist):
        self.send("set_speed", 8, [twist.linear.x, twist.linear.y, twist.angular.z])

    def parse_data(self, data):
        data_splitted = data.data.split()
        action_name = data_splitted[0]
        action_type = int(data_splitted[1])
        args_str = data_splitted[2:]
        action_args_dict = {'B': int, 'H': int, 'f': float}
        args = [action_args_dict[t](s) for t, s in zip(self.pack_format[action_type][1:], args_str)]
        return action_name, action_type, args

    def finish_command(self, action_name, action_status="finished"):
        rospy.loginfo("FINISHED STM COMMAND: " + action_name + " with status " + action_status)
        self.pub_response.publish(action_name + " " + action_status)

    def stm_command_callback(self, data):
        action_name, action_type, args = self.parse_data(data)
        successfully, responses = self.send(action_name, action_type, args)
        if action_type in IMMEDIATE_FINISHED:
            rospy.loginfo("start immediate finished " + action_name + " " + str(action_type) + " " + str(args))
            self.finish_command(action_name, "finished")
        if action_type in DEBUG_COMMANDS:
            rospy.loginfo(action_name + ' ' + str(action_type) + ' ' + str(args) + ' ' + "successfully? :" +
                          str(successfully) + ' ' + str(responses))
        if action_type in ODOMETRY_MOVEMENT or \
                (action_type == CMD_VEL and args == [.0, .0, .0]):
            rospy.loginfo("start odometry movement " + action_name + " " + str(action_type) + " " + str(args))
            self.odometry_movement_id = action_name
            self.timer_odom_move = rospy.Timer(rospy.Duration(1.0 / STATUS_RATE), self.odometry_movement_timer)
        if action_type in MANIPULATOR_JOBS:
            rospy.loginfo("start manipulator jobs " + action_name + " " + str(action_type) + " " + str(args))
            if self.robot_name == "main_robot":
                n = args[0]
            else:
                n = action_type - 0xc1  # first dynamixel
            self.take_cube[n] = action_name
            self.timer_m[n] = rospy.Timer(rospy.Duration(1.0 / STATUS_RATE),
                                          self.manipulator_timer(n, GET_SEC_ROBOT_MANIPULATOR_STATUS if action_type
                                          in range(0xc1, 0xc4) else GET_MANIPULATOR_STATUS))

    def send(self, action_name, action_type, args):
        self.mutex.acquire()
        successfully, args_response = self.send_command(action_type, args)
        self.mutex.release()
        return successfully, args_response

    def publish_odom(self, coords):
        # check if no nan values
        if np.any(coords != coords):
            return

        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "%s_odom" % self.robot_name
        t.child_frame_id = self.robot_name
        t.transform.translation.x = coords[0]
        t.transform.translation.y = coords[1]
        t.transform.translation.z = 0.0
        q = tf_conversions.transformations.quaternion_from_euler(0, 0, coords[2])
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.br.sendTransform(t)

        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = self.robot_name
        t.child_frame_id = "%s_laser" % self.robot_name
        t.transform.translation.x = self.laser_coords[0]
        t.transform.translation.y = self.laser_coords[1]
        t.transform.translation.z = .4
        q = tf_conversions.transformations.quaternion_from_euler(0, 0, self.laser_angle)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.br.sendTransform(t)

    def pub_odom_coords_timer_callback(self, event):
        successfully, coords = self.send('request_stm_coords', 15, [])
        if successfully:
            self.publish_odom(coords)

    def pub_rf_data_timer_callback(self, event):
        if self.robot_name == "main_robot":
            successfully, rf_data = self.send('request_rf_data', REQUEST_RF_DATA, [])
            if successfully:
                self.pub_rf.publish(Int32MultiArray(data=rf_data))
            else:
                rospy.loginfo("RF response " + str(successfully))

        if self.robot_name == "secondary_robot":
            successfully, rf_data = self.send('request_rf_data_secondary_robot', REQUEST_RF_DATA_SECONDARY, [])
            if successfully:
                self.pub_rf.publish(Int32MultiArray(data=rf_data))
            else:
                rospy.loginfo("RF response " + str(successfully))

    def odometry_movement_timer(self, event):
        successfully, args_response = self.send('GET_ODOMETRY_MOVEMENT_STATUS', GET_ODOMETRY_MOVEMENT_STATUS, [])
        if successfully:
            # finished
            if args_response[0] == 0:
                self.finish_command(self.odometry_movement_id)
                self.timer_odom_move.shutdown()

    def manipulator_timer(self, n, command_name=GET_MANIPULATOR_STATUS):
        def m_timer(event):
            successfully, args_response = self.send('GET_MANIPULATOR_' + str(n) + '_STATUS', command_name,
                                                    [n] if command_name == GET_MANIPULATOR_STATUS else [])
            if successfully:
                # status code: 0 - done; 1 - in progress; >1 - error
                status = args_response[0]
                # if error 
                if status > 1:
                    rospy.logerr("Manipulator " + str(n) + " error. Code: " + str(status))
                    self.finish_command(self.take_cube[n], 'error ' + str(status))
                    self.timer_m[n].shutdown()
                # if action is finished
                elif status == 0:
                    self.finish_command(self.take_cube[n])
                    self.timer_m[n].shutdown()

        return m_timer

    def stm_node_command_callback(self, data):
        rospy.loginfo("stm node command " + data.data)
        splitted_data = data.data.split()
        if splitted_data[1] == "start_wire":
            self.wire_timer.shutdown()
            self.wire_timer = rospy.Timer(rospy.Duration(1. / 30), self.wire_timer_callback)
        elif splitted_data[1] == "stop_wire":
            # self.wire_timer.shutdown()
            pass

    def wire_timer_callback(self, event):
        successfully, args_response = self.send('GET_STARTUP_STATUS', GET_STARTUP_STATUS, [])
        if successfully:
            self.pub_wire.publish(str(args_response[0]))


if __name__ == '__main__':
    stm_serial_port = "/dev/ttyUSB0"
    stm = StmNode(stm_serial_port)
    rospy.spin()
