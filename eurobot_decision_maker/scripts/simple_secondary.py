import rospy
import tf
import time
import numpy as np
from std_msgs.msg import String


class Simple_strategy():
    def __init__(self, stm_pub, res_sub):
        self.stm_pub = stm_pub
        self.rate = rospy.Rate(30)
        self.listener = tf.TransformListener()
        self.id = 0
        self.is_response = False
        rospy.Subscriber(res_sub, String, self.response_callback)

    def get_next_id(self):
        self.id += 1
        return self.id

    def start_stm_cmd(self, cmd, with_response=True):
        self.get_next_id()
        self.is_response = False
        self.stm_pub.publish(str(self.id) + " " + cmd)
        if not with_response:
            while not self.is_response and not rospy.is_shutdown():
                self.rate.sleep()

    def take_angle(self):
        (trans, rot) = self.listener.lookupTransform('map', 'secondary_robot', rospy.Time(0))
        yaw = tf.transformations.euler_from_quaternion(rot)[2]
        return yaw

    def take_coord(self):
        (trans, rot) = self.listener.lookupTransform('map', 'secondary_robot', rospy.Time(0))
        return np.array(trans[:2])

    def move_on_angle(self, target_angle):
        self_angle = self.take_angle()
        da = (target_angle - self_angle) % (2 * np.pi)
        if da > np.pi:
            da = np.pi - da
        else:
            da = da
        self.start_stm_cmd("0 0 " + str(da) + " 0 0 1")

    def response_callback(self, data):
        data_splitted = data.data.split()
        if data_splitted[0] == str(self.id) and data_splitted[1] == "finished":
            self.is_response = True

    def move_on_point(self, target_point):
        self_point = self.take_coord()
        angle = self.take_angle()
        dl = target_point - self_point
        dl[0], dl[1] = dl[0] * np.cos(angle) + dl[1] * np.sin(angle), -dl[0] * np.sin(angle) + dl[1] * np.cos(angle)
        dt = np.max(dl / np.array([0.2, 0.2]))
        v = np.abs(dl / dt)
        self.start_stm_cmd(str(dl[0]) + " " + str(dl[1]) + " 0 " + str(v[0]) + " " + str(v[1]) + " 0")

    def start(self):
        self.move_on_angle(4.71)
        self.move_on_point(np.array([0.2, 0.5]))
        self.start_stm_cmd("8 -0.1 0 0")
        time.sleep(4)
        self.start_stm_cmd("8 0 0 0")

