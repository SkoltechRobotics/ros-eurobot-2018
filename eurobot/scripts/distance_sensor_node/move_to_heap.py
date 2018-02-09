#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import numpy as np


def command_callback(data):
    data_splitted = data.data.split()
    action_type = data_splitted[1]
    print("Receive command " + data.data)
    if action_type == "MOVETOHEAP":
        pass


def distance_sensors_callback(data):
    global sensors
    global start_sensors
    try:
        sensors1 = map(int, data.data.split())
    except ValueError:
        sensors1 = start_sensors

    sensors = sensors


if __name__ == '__main__':
    try:
        sensors = np.zeros(4)
        start_sensors = np.array([35, 85, 85, 35])
        rospy.init_node('read_data_node', anonymous=True)

        pub_command = rospy.Publisher("/main_robot/stm_command", String, queue_size=10)
        rospy.Subscriber("/main_robot/move_command", String, command_callback)
        rospy.Subscriber("distance_sensors/distances", String, distance_sensors_callback)


        rospy.spin()
    except rospy.ROSInterruptException:
        pass
