#!/usr/bin/env python
import cv2
import rospy
import cv_bridge
from sensor_msgs.msg import Image
from PlanRecognition import find_colors, clh_transform, STEP, get_rough
import numpy as np

COLORS = np.array([[0, 124, 176], [208, 93, 40], [14, 14, 16], [97, 153, 59],
                   [247, 181, 0]], dtype=np.uint8)
img_points = np.float32([(798, 549), (798, 488), (912, 487), (912, 553)])
h_border = STEP * 2 * 3
w_border = STEP * 2 * 7
h_rect = int(130 / 30 * STEP)
w_rect = int(300 / 30 * STEP)
dw = (w_border - w_rect) / 2
dh = (h_border - h_rect) / 2
real_points = np.float32([(dw, h_border - dh), (dw, dh),
                          (w_border - dw, dh), (w_border - dw, h_border - dh)])
M = cv2.getPerspectiveTransform(img_points, real_points)


def img_callback(data):
    global bridge
    global pub
    img = bridge.imgmsg_to_cv2(data, "bgr8")
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    img = cv2.warpPerspective(img, M, (w_border, h_border))

    colors, ind = find_colors(img)
    img = clh_transform(img)
    rough_img = get_rough(img)
    for i, color in enumerate(colors):
        cv2.rectangle(img, ((ind[1] + 2 * i) * STEP, ind[0] * STEP), ((ind[1] + 2 * i) * STEP + STEP,
                                                                      ind[0] * STEP + STEP),
                      [int(x_) for x_ in COLORS[color]], STEP // 6 + 1 if STEP < 6 else 0)

    img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    rough_img = cv2.cvtColor(rough_img, cv2.COLOR_RGB2BGR)
    pub.publish(bridge.cv2_to_imgmsg(img, "bgr8"))
    pub_add_img.publish(bridge.cv2_to_imgmsg(rough_img, "bgr8"))


if __name__ == '__main__':
    rospy.init_node('img_node', anonymous=True)
    pub = rospy.Publisher("/usb_cam/image_tr", Image, queue_size=3)
    pub_add_img = rospy.Publisher("usb_cam/add_image", Image, queue_size=3)
    rospy.Subscriber("/usb_cam/image_1", Image, img_callback)

    bridge = cv_bridge.CvBridge()
    rospy.spin()
