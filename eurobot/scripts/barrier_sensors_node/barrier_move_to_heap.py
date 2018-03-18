#!/usr/bin/env python
import rospy
import numpy as np
import serial
from std_msgs.msg import Int32MultiArray


class BarrierNavigator():
    def __init__(self, stm_command_publisher_name, response_pub_name):
        self.sensors = []
        self.phase = 0
        self.sensors_goals = []
        self.dx_quant = 0.002
        self.dy_quant = 0.002
        self.speed_xy = 0.1
        self.speed_z  = 0.1
        self.command_name = str(0xa2)
        self.ready = True
        self.command_pub_name = stm_command_publisher_name
        self.command_pub = rospy.Publisher(stm_command_publisher_name, String, queue=10)
        self.response_pub_name = response_pub_name
        self.response_pub = rospy.Publisher(response_pub_name, String, queue=2)



    def barrier_sensors_callback(self):
        def cb(data):
            self.sensors = np.array(data.data)
        return cb

    def start_command_callback(self):
        def cb(data):
            data_splitted = data.data.split()
            action_type = data_splitted[1]
            rospy.loginfo("Receive command " + data.data)

            if action_type == "MOVETOHEAP":
                config = int(data_splitted[2])
                rospy.sleep(0.5)
                rospy.loginfo("Start move to heap by barrier sensors")

                self.set_sensors_goals()

                while not rospy.is_shutdown() and np.any(self.sensors - self.sensors_goals != 0):
                    self.command_pub.publish(self.get_command())
                    self.wait_for_movement()

                self.response_pub.publish(data_splitted[0] + ' finished')

    def wait_for_movement(self):
        while not rospy.is_shutdown():
            msg = rospy.wait_for_message(self.response_pub_name, String, timeout=2)
            if msg.data == "MOVEODOM finished":
                break

    def set_sensors_goals(self):
        #if self.phase == 0:
        return np.zeros((6),dtype=np.int)

    def get_command(self):
        dx = self.sensors[5]*self.dx_quant - (self.sensors[3] or self.sensors[4])*self.dx_quant
        dy = np.any(self.sensors[:3])*self.dy_quant
        dz = 0
        command_string = "MOVEODOM " + self.command_name + ' '
        command_string += str(dx) + ' ' + str(dy) + ' ' + str(dz) + ' '
        command_string += str(self.speed_xy) + ' ' + str(self.speed_xy) + ' ' + str(self.speed_z)
        return command_string



def get_sensor_goal_values(sensor_data):
    # 0 - move to y+ (biggest x)
    # 1 - move to y+ (middle x)
    # 2 - move to y+ (smallest x)
    # 3 - move to x- (0)
    # 4 - move to x- (1)
    # 5 - move to x+ (2)


if __name__ == '__main__':
    try:
        rospy.init_node('barrier_move_node', anonymous=True)

        pub_command = rospy.Publisher("/main_robot/stm_command", String, queue_size=10)
        rospy.Subscriber("/main_robot/move_command", String, command_callback)
        pub_response = rospy.Publisher("/main_robot/response", String, queue_size=2)


    except rospy.ROSInterruptException:
        pass
