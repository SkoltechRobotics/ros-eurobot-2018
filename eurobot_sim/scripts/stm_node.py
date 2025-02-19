#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import numpy as np
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


class stm_node():
    def __init__(self):
        # ROS
        rospy.init_node('stm_node', anonymous=True)
        rospy.Subscriber("stm_command", String, self.stm_command_callback)
        rospy.Subscriber("cmd_vel", Twist, self.set_twist)
        self.pub_response = rospy.Publisher("response", String, queue_size=10)
        self.pub_odom = rospy.Publisher("odom", Odometry, queue_size=1)

        self.robot_name = rospy.get_param('robot_name')
        self.br = tf.TransformBroadcaster()

        # high-level commands info (for handling response)
        self.actions_in_progress = ['']  # action_names, indexing corresponds to types indexing
        self.action_types = []  # list of high-level action types only
        self.actions_with_response = [0x0E]

        self.pack_format = {
            0x01: "=BBBB",
            0x03: "=Bf",
            0x04: "=B",
            0x05: "=B",
            0x08: "=fff",
            0x09: "=",
            0x0a: "=",
            0x0b: "=BH",
            0x0c: "=B",
            0x0d: "=B",
            0xa0: "=fff",
            0xa1: "=fff",
            0xb0: "=B",
            0xc0: "=BB",
            0xb1: "=B",
            0x0e: "=fff",
            0x0f: "=",
        }

        self.unpack_format = {
            0x01: "=BBBB",
            0x03: "=BB",
            0x04: "=BB",
            0x05: "=BB",
            0x08: "=BB",
            0x09: "=fff",
            0x0a: "=fff",
            0x0b: "=BB",
            0x0c: "=f",
            0x0d: "=BB",
            0xa0: "=Bfff",
            0xa1: "=BB",
            0xb0: "=BB",
            0xc0: "=BB",
            0xb1: "=BB",
            0x0e: "=BB",
            0x0f: "=fff",
        }

        self.freq = 400
        self.rate = rospy.Rate(self.freq)  # 100Hz

        self.color = rospy.get_param("/field/color")

        self.coords = np.array(rospy.get_param('start_' + self.color))
        self.coords[:2] /= 1000.0
        self.laser_coords = (rospy.get_param('lidar_x') / 1000.0, rospy.get_param('lidar_y') / 1000.0, 0.41)
        self.vel = np.array([0.0, 0.0, 0.0])

        self.last_integration_time = rospy.get_time()
        rospy.Timer(rospy.Duration(1. / 80), self.pub_timer_callback)

    def set_twist(self, twist):
        vel = np.zeros(3)
        vel[0] = twist.linear.x
        vel[1] = twist.linear.y
        vel[2] = twist.angular.z
        self.set_vel(vel)

    def parse_data(self, data):
        data_splitted = data.data.split()
        action_name = data_splitted[0]
        action_type = int(data_splitted[1])
        args_str = data_splitted[2:]
        # TBD: split any chars in Strings like 'ECHO'->['E','C','H','O']
        action_args_dict = {'B': ord, 'H': int, 'f': float}
        args = [action_args_dict[t](s) for t, s in zip(self.pack_format[action_type][1:], args_str)]
        return action_name, action_type, args

    def set_vel(self, vel):
        self.vel = vel

    def set_coords(self, coords):
        self.coords = coords

    def stm_command_callback(self, data):
        # parse data
        action_name, action_type, args = self.parse_data(data)
        rospy.loginfo(str(action_type))

        # simulate STM32 response
        successfuly = True
        args_response = "Ok"
        if action_type == 0x08:
            self.set_vel(np.array(args))
        elif action_type == 0x09:
            args_response = self.vel
        elif action_type == 0x0E:
            self.set_coords(np.array(args))
        elif action_type == 0x0F:
            args_response = self.coords

        # high-level commands handling
        if action_type in self.action_types:
            # store action_name
            # rospy.loginfo(str(action_type))
            self.actions_in_progress[self.action_types[action_type]] = action_name

        # low-level commands handling
        elif action_type in self.actions_with_response:
            rospy.loginfo(action_name + " finished")

            def delayed_cb(e):
                self.pub_response.publish(action_name + " finished")

            rospy.Timer(rospy.Duration(0.2), delayed_cb, oneshot=True)

    def handle_response(self, status):
        """Handles response for high-lvl commands (only)."""
        l = len(status)
        for i in range(l):
            # mind that indeces in status[] correspond to indeces in actions_in_progress[]
            rospy.loginfo(status[i] + ' ' + str(self.action_in_progress[i]))
            if status[i] == '0' and len(self.actions_in_progress[i]) > 0:
                self.actions_in_progress[i] = ''  # stop storing this action_name
                self.pub_response.publish(self.actions_in_progress[i] + " done")  # publish responce

            self.rate.sleep()

    def integrate(self):
        while not rospy.is_shutdown():
            # time between iterations
            now = rospy.get_time()
            dt = now - self.last_integration_time
            self.last_integration_time = now

            # matrix for conveting into world frame
            a = self.coords[2]
            M = np.array([[np.cos(a), -np.sin(a)],
                          [np.sin(a), np.cos(a)]])  # TODO
            vel = self.vel.copy()
            vel[:2] = np.matmul(M, vel[:2].reshape((2, 1))).reshape((2,))

            # process noise
            noise = np.random.normal(size=3) * 0.1 * vel
            # noise *= 0.96 # simulate bad estimation of wheel size, etc.

            # add dx and noise to coords
            self.coords += (vel + noise) * dt
            self.coords[2] = self.coords[2] % (2 * np.pi)
            self.rate.sleep()

    def pub_timer_callback(self, event):
        # localization noise
        coords = self.coords.copy()
        coords[:2] += np.random.normal(size=2, scale=0.0005)

        odom = Odometry()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = self.robot_name
        odom.pose.pose.position.x = coords[0]
        odom.pose.pose.position.y = coords[1]
        quat = tf.transformations.quaternion_from_euler(0, 0, coords[2])
        odom.pose.pose.orientation.z = quat[2]
        odom.pose.pose.orientation.w = quat[3]
        odom.twist.twist.linear.x = self.vel[0]
        odom.twist.twist.linear.x = self.vel[1]
        odom.twist.twist.angular.z = self.vel[2]
        self.pub_odom.publish(odom)

        self.br.sendTransform((coords[0], coords[1], 0),
                              tf.transformations.quaternion_from_euler(0, 0, coords[2]),
                              rospy.Time.now(),
                              self.robot_name,
                              "%s_odom" % self.robot_name)

        self.br.sendTransform(self.laser_coords,
                              tf.transformations.quaternion_from_euler(0, 0, 1.570796),
                              rospy.Time.now(),
                              '%s_laser' % self.robot_name,
                              self.robot_name)


if __name__ == '__main__':
    try:
        stm = stm_node()

        stm.integrate()
    except rospy.ROSInterruptException:
        pass
