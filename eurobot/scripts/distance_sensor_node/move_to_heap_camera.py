#!/usr/bin/env python
import rospy
import sys
from std_msgs.msg import String
import numpy as np
from std_msgs.msg import Float32MultiArray
import time
import tf


def get_coordinates(data):
    data_splitted = data.data.split()
    global coord
    coord = np.array([float(data_splitted[0]), float(data_splitted[1])])


def command_callback(data):
    global coord
    data_splitted = data.data.split()
    action_type = data_splitted[1]
    rospy.loginfo("Receive command " + data.data)

    if action_type == "MOVETOHEAP":
        config = int(data_splitted[2])
        rospy.sleep(0.5)
        rospy.loginfo("Start move to heap by camera")
        x, y = coord / 1000.
        rospy.loginfo("coordinates of heap is " + str(x) + " " + str(y))
        pub_command.publish("MOVE 162 " + str(x) + " " + str(y) + " 0 0.2 0.2 0")


if __name__ == '__main__':
    try:
        coord = np.array([0, 0])
        rospy.init_node('move_to_heap_camera', anonymous=True)

        pub_command = rospy.Publisher("/main_robot/stm_command", String, queue_size=10)
        rospy.Subscriber("/main_robot/move_command", String, command_callback)
        rospy.Subscriber("/usb_cam/coordinates", String, get_coordinates)
        pub_response = rospy.Publisher("/main_robot/response", String, queue_size=2)

        rospy.spin()
    except rospy.ROSInterruptException:
        pass
