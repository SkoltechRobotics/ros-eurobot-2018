#!/usr/bin/env python
import rospy
import numpy as np
import serial
from std_msgs.msg import Float32MultiArray


if __name__ == '__main__':
    try:
        rospy.init_node('read_data_node', anonymous=True)
        pub_raw = rospy.Publisher('distance_sensors/distances/raw', Float32MultiArray, queue_size=2)
        pub_smooth = rospy.Publisher('distance_sensors/distances/smooth', Float32MultiArray, queue_size=2)
        ser = serial.Serial("/dev/ttyACM0")

        rospy.loginfo("Connect to /dev/ttyACM0 for rangefinder data successfully")
        a = 0.8
        rospy.loginfo("Smooth value for exponential filter is " + str(a))
        sensors_smooth = np.array([255, 255, 255, 255, 255])
        while not rospy.is_shutdown():
            s = ser.readline()
            try:
                sensors_raw = np.array(map(int, s.split()))
            except ValueError:
                pass
            else:
                if sensors_raw.shape[0] == 5:
                    pub_raw.publish(Float32MultiArray(data=sensors_raw))
                    sensors_smooth = a * sensors_smooth + (1 - a) * sensors_raw
                    pub_smooth.publish(Float32MultiArray(data=sensors_smooth))
    except rospy.ROSInterruptException:
        pass
