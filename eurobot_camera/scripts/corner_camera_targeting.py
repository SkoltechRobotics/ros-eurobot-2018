#!/usr/bin/env python
import cv2
import rospy
import cv_bridge
from sensor_msgs.msg import Image
from std_msgs.msg import String
from PlanRecognition import find_colors_geom, img_transformation, STEP, rag
import numpy as np

COLORS = np.array([[0, 124, 176], [208, 93, 40], [14, 14, 16], [97, 153, 59],
                   [247, 181, 0]], dtype=np.uint8)
# img_points = np.float32([(798, 549), (798, 488), (912, 487), (912, 553)])
COLOR_NAMES = ["blue", "orange", "black", "green", "yellow"]
img_points = np.float32([(950, 610), (950, 550), (1062, 550), (1062, 610)])
h_border = STEP * 2 * 3
w_border = STEP * 2 * 7
h_rect = int(130 / 30 * STEP)
w_rect = int(300 / 30 * STEP)
dw = (w_border - w_rect) / 2
dh = (h_border - h_rect) / 2
real_points = np.float32([(dw, h_border - dh), (dw, dh),
                          (w_border - dw, dh), (w_border - dw, h_border - dh)])
M = cv2.getPerspectiveTransform(img_points, real_points)

params = {"kl": 2,
          "kp": 1,
          "k1": 10,
          "k2": 1,
          "k3": 1.1,
          "kr": 0.93,
          "gaus_sigma": 2,
          "r_disk": 4,
          "thresh": 30,
          "compactness": 20,
          "s_cutoff": 0.27,
          "s_gain": 30,
          "v_cutoff": 0.25,
          "v_gain": 30,
          "c": -0.45}


def img_callback(img):
    global bridge
    global pub
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    img = cv2.warpPerspective(img, M, (w_border, h_border))

    img1 = img_transformation(img, **params)
    img2 = rag(img1, **params)[0]
    pub_add_img.publish(bridge.cv2_to_imgmsg(cv2.cvtColor(img1, cv2.COLOR_RGB2BGR), "bgr8"))

    colors, _, centers = find_colors_geom(img, **params)
    centers = np.array(centers).T.astype(np.uint8)
    img = img2
    for i, color in enumerate(colors):
        cv2.rectangle(img, (centers[i, 0] + STEP // 2, centers[i, 1] + STEP // 2),
                           (centers[i, 0] - STEP // 2, centers[i, 1] - STEP // 2),
                      [int(x_) for x_ in COLORS[color]], STEP // 6 + 1 if STEP < 6 else 0)

    img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    pub.publish(bridge.cv2_to_imgmsg(img, "bgr8"))
    return colors


def cmd_callback(data):
    global is_start
    global is_finish
    data_splitted = data.data.split()
    if data_splitted[1] == "start":
        is_start = True
    if data_splitted[1] == "finish":
        is_finish = True


if __name__ == '__main__':
    rospy.init_node('img_node', anonymous=True)
    pub = rospy.Publisher("/usb_cam/result_img", Image, queue_size=3)
    pub_add_img = rospy.Publisher("usb_cam/add_image", Image, queue_size=3)
    pub_plan = rospy.Publisher("/server/plan", String, queue_size=3)
    rospy.Subscriber("/server/camera_command", String, cmd_callback)
    bridge = cv_bridge.CvBridge()
    rate = rospy.Rate(100)

    rospy.loginfo("Start camera node")
    is_start = False
    while not is_start and not rospy.is_shutdown():
        rate.sleep()
    cap = cv2.VideoCapture(2)
    cap.set(cv2.CAP_PROP_FPS, 10)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1600)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1200)
    rospy.loginfo("Start capture video")
    is_finish = False
    while not rospy.is_shutdown() and not is_finish:
        ret, frame = cap.read()
        colors = img_callback(frame)
        color_str = COLOR_NAMES[colors[0]] + " " + COLOR_NAMES[colors[1]] + " " + COLOR_NAMES[colors[2]]
        pub_plan.publish(color_str)
        rospy.loginfo("colors " + color_str)