#!/usr/bin/env python
import rospy
import numpy as np
import serial
from std_msgs.msg import Float32MultiArray


if __name__ == '__main__':
    try:
        rospy.init_node('read_data_node', anonymous=True)
        pub_raw = rospy.Publisher('distance_sensors/distances/raw', Float32MultiArray, queue_size=2)
        pub_smooth = rospy.Publisher('distance_sensors/distances/smooth', Float32MultiArray, queue_size=3)
        ser = serial.Serial("/dev/ttyACM0", timeout=0.2)

        rospy.loginfo("Connect to /dev/ttyACM0 for rangefinder data successfully")
        rospy.sleep(2.2)
        n = 4
        sensors_smooth = np.ones((n, 5)) * 255
        #window = np.array([0.0083, 0.0462, 0.1115, 0.1115, 0.0462, 0.0083])
        #window = np.array([0.0083, 0.0462, 0.1115])
        window = np.ones(n)
        window /= np.sum(window)
        while not rospy.is_shutdown():
            s = ser.readline()
            if len(s) == 0:
                rospy.logwarn("Serial port doesn't read any rangefinders data")
            try:
                sensors_raw = np.array(map(float, s.split()), dtype=np.float32)
            except ValueError:
                pass
            else:
                if sensors_raw.shape[0] == 5:
                    pub_raw.publish(Float32MultiArray(data=sensors_raw))
                    sensors_smooth = np.roll(sensors_smooth, -1, axis=0)
                    sensors_smooth[n - 1] = sensors_raw
                    smooth_data = (window[np.newaxis, :].dot(sensors_smooth))[0]
                    pub_smooth.publish(Float32MultiArray(data=smooth_data))
    except rospy.ROSInterruptException:
        pass
