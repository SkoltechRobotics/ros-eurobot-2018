#!/usr/bin/env python
import cv2
import rospy
import cv_bridge
from sensor_msgs.msg import Image
import numpy as np
from EnemiesRecognition import CAMERA_MATRIX_0, CAMERA_MATRIX_45, K, D, find_enemy


def img_callback(data):
    global bridge
    global pub_0
    global pub_45
    global is_first
    global first_img
    global true_img
    raw_img = bridge.imgmsg_to_cv2(data, "bgr8")

    pub_0.publish(bridge.cv2_to_imgmsg(cv2.fisheye.undistortImage(raw_img, K, D, Knew=CAMERA_MATRIX_0), "bgr8"))

    img = cv2.fisheye.undistortImage(raw_img, K, D, Knew=CAMERA_MATRIX_45)
    if is_first:
        is_first = False
        first_img = img
        return

    # img = (img / 255. - first_img / 255. / 2.).clip(0, 1)
    # img = (255 * img).astype(np.uint8)
    ind = find_enemy(img)

    cv2.circle(img, (ind[1], ind[0]), 50, [0, 0, 255], 10)
    pub_45.publish(bridge.cv2_to_imgmsg(img, "bgr8"))

    result_img = true_img.copy()
    cv2.circle(result_img, (ind[1], ind[0]), 50, [0, 0, 255], 10)
    pub_result.publish(bridge.cv2_to_imgmsg(result_img, "bgr8"))


if __name__ == '__main__':
    rospy.init_node('central_camera_node', anonymous=True)

    true_img_raw = cv2.imread("/home/mikhail/Field.png")
    rows, cols, ch = (960, 1280, 3)
    rows1, cols1, ch = true_img_raw.shape

    pts1 = np.float32([[0, 0], [0, rows1], [cols1, 0]])
    pts2 = np.float32([[cols, rows], [cols, 0], [0, rows]])

    M_t = cv2.getAffineTransform(pts1, pts2)

    true_img = cv2.warpAffine(true_img_raw, M_t, (cols, rows))

    bridge = cv_bridge.CvBridge()
    first_img = None
    is_first = True
    pub_0 = rospy.Publisher("/usb_cam/on_ground", Image, queue_size=3)
    pub_45 = rospy.Publisher("/usb_cam/45_ground", Image, queue_size=3)
    pub_result = rospy.Publisher("/usb_cam/enemies", Image, queue_size=3)
    rospy.Subscriber("/usb_cam/image_2", Image, img_callback)

    rospy.spin()
