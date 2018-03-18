#!/usr/bin/env python
import cv2
import rospy
import cv_bridge
from sensor_msgs.msg import Image
from std_msgs.msg import String
import numpy as np
from EnemiesRecognition import K, D
from CubeLocalisation import search_cube


K1 = K.copy()
K1[:2] /= 4
K2 = K1.copy()
K2[:2, :2] /= 2
CENTER = (160, 120)
KY = -6.5
X0 = 323
KX = -3.
Y0 = 173.5


def img_callback(data):
    raw_img = bridge.imgmsg_to_cv2(data, "bgr8")

    img = cv2.fisheye.undistortImage(raw_img, K1, D, Knew=K2)
    params, edge = search_cube(img)

    cv2.rectangle(img, (params[3], params[1]), (params[2], params[0]), (255, 0, 0))
    pub_0.publish(bridge.cv2_to_imgmsg(img, "bgr8"))
    pub_1.publish(bridge.cv2_to_imgmsg(np.repeat((255 * edge[:, :, np.newaxis]).astype(np.uint8), 3, axis=2), "bgr8"))
    x = (params[2] + params[3] - X0) / KX
    y = (params[1] - params[0] + params[3] - params[2] - Y0) / KY
    pub_coord.publish(str(x) + " " + str(y))


if __name__ == '__main__':
    rospy.init_node('big_robot_camera_node', anonymous=True)
    bridge = cv_bridge.CvBridge()
    pub_0 = rospy.Publisher("/usb_cam/result_img", Image, queue_size=3)
    pub_1 = rospy.Publisher("/usb_cam/result_img2", Image, queue_size=3)
    rospy.Subscriber("/usb_cam/image_raw", Image, img_callback)
    pub_coord = rospy.Publisher("/usb_cam/coordinates", String, queue_size=3)

    rospy.spin()
