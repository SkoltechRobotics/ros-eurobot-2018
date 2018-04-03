#!/usr/bin/env python
import rospy
import numpy as np
import serial
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import String

if __name__ == '__main__':
    try:
        rospy.init_node('barrier_move_node', anonymous=True)

        # pub_command = rospy.Publisher("/main_robot/stm_command", String, queue_size=10)
        # rospy.Subscriber("/main_robot/move_command", String, command_callback)
        # pub_response = rospy.Publisher("/main_robot/response", String, queue_size=2)
        pub_movement = rospy.Publisher("/main_robot/barrier_movement", Int32MultiArray, queue_size=2)

        ser = serial.Serial("/dev/ttyACM0", timeout=0.2)

        rospy.loginfo("Connect to /dev/ttyACM0 for barrier data successfully")
        rospy.sleep(2.2)
        while not rospy.is_shutdown():
            s = ser.readline()
            if len(s) == 0:
                rospy.logwarn("Serial port doesn't read any rangefinders data")
            try:
                sensors_raw = np.array(map(int, s.split()), dtype=np.float32)
            except ValueError:
                pass
            else:
                # rospy.loginfo(sensors_raw.shape)
                if sensors_raw.shape[0] == 6:
                    pub_movement.publish(Int32MultiArray(data=sensors_raw))

    except rospy.ROSInterruptException:
        pass
