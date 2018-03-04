#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import serial


if __name__ == '__main__':
    try:
        rospy.init_node('read_data_node', anonymous=True)
        pub = rospy.Publisher('distance_sensors/distances', String, queue_size=2)

        ser = serial.Serial("/dev/ttyACM0")
        while not rospy.is_shutdown():
            s = ser.readline()
            pub.publish(s)
    except rospy.ROSInterruptException:
        pass
