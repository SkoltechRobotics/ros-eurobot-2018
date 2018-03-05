#!/usr/bin/env python
import io
import picamera
import cv2
import rospy
import cv_bridge
from sensor_msgs.msg import Image
import numpy as np
import time
from picamera.array import PiRGBArray   

if __name__ == '__main__':
    rospy.init_node('img_publisher', anonymous=True)
    img_pub = rospy.Publisher("image", Image, queue_size=1)

    camera = picamera.PiCamera()
    camera.resolution = (1440, 1080)
    bridge = cv_bridge.CvBridge()

    rate = rospy.Rate(20)

    start_time = time.time()
    rawCapture = PiRGBArray(camera)
    for i in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True): 
        print("1", time.time() - start_time)
        img = rawCapture.array
        rawCapture.truncate(0)
        print("2", time.time() - start_time)
        img_pub.publish(bridge.cv2_to_imgmsg(img, "bgr8"))
        print("3", time.time() - start_time)
        rate.sleep()
        print("--------------------------------")
        start_time = time.time()
        if rospy.is_shutdown():
            break
