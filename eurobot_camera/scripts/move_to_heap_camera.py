#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import numpy as np
import cv_bridge
import cv2
from EnemiesRecognition import K, D
from CubeLocalisation import search_cube
from sensor_msgs.msg import Image
import tf


K1 = K.copy()
K1[:2] /= 4
K2 = K1.copy()
K2[:2, :2] /= 2
CENTER = (160, 120)
KY = -6.5 / 2.
X0 = 323
KX = -3.
Y0 = 173.5 / 2.
MIN_DELTA = 0.002
MAX_DISTANCE = 0.03
MIN_ANGLE = 0.01


def img_callback(data):
    global raw_img
    raw_img = bridge.imgmsg_to_cv2(data, "bgr8")


def response_callback(data):
    global is_moved
    data_splitted = data.data.split()
    if data_splitted[0] == "move_heap_121":
        is_moved = True


def command_callback(data):
    global is_moved
    global raw_image
    data_splitted = data.data.split()
    action_type = data_splitted[1]
    rospy.loginfo("Receive command " + data.data)

    if action_type == "MOVETOHEAP":
        # config = int(data_splitted[2])
        rospy.sleep(0.8)
        rospy.loginfo("Start move to heap by camera")

        x, y = MIN_DELTA + 1e-4, MIN_DELTA + 1e-4

        (trans, rot) = listener.lookupTransform('/map', '/main_robot', rospy.Time(0))
        yaw = tf.transformations.euler_from_quaternion(rot)[2]
        yaw = yaw % (np.pi / 2)
        a = - (yaw if yaw < np.pi / 4 else yaw - np.pi / 2)

        is_moved = False
        pub_command.publish("move_heap_121 162 0 0 " + str(a) + ' 0 0 0.5')
        while not is_moved:
            rate.sleep()
        rospy.sleep(0.8)

        while True:
            img = cv2.fisheye.undistortImage(raw_img, K1, D, Knew=K2)
            params, edge = search_cube(img)

            cv2.rectangle(img, (params[3], params[1]), (params[2], params[0]), (255, 0, 0))
            pub_0.publish(bridge.cv2_to_imgmsg(img, "bgr8"))

            x = (params[2] + params[3] - X0) / KX
            y = (params[3] - params[2] - Y0) / KY
            x, y = x / 1000., y / 1000.

            if abs(x) < MIN_DELTA and abs(y) < MIN_DELTA:
                break

            if abs(x) > MAX_DISTANCE or abs(y) > MAX_DISTANCE:
                rospy.logerr("Not correct movement " + str(x) + " " + str(y) + " " + str(a))
                return
            rospy.loginfo("coordinates of heap is " + str(x) + " " + str(y) + " " + str(a))
            is_moved = False
            v_max = np.array([0.2, 0.2, 0.5])
            point = np.array([x, y, 0])
            v = np.abs(point / np.max(np.abs(point) / v_max))
            pub_command.publish("move_heap_121 162 " + ' '.join(map(str, point)) + " " + ' '.join(map(str, v)))

            while not is_moved:
                rate.sleep()
            rospy.sleep(1.2)
        pub_response.publish(data_splitted[0] + " finished")


if __name__ == '__main__':
    try:
        rospy.init_node('move_to_heap_camera', anonymous=True)

        bridge = cv_bridge.CvBridge()
        raw_image = 0
        is_moved = False
        rate = rospy.Rate(20)
        listener = tf.TransformListener()

        pub_command = rospy.Publisher("/main_robot/stm_command", String, queue_size=10)
        pub_response = rospy.Publisher("/main_robot/response", String, queue_size=2)
        pub_0 = rospy.Publisher("/usb_cam/result_img", Image, queue_size=3)

        rospy.Subscriber("/main_robot/move_command", String, command_callback)
        rospy.Subscriber("/usb_cam/image_raw", Image, img_callback)
        rospy.Subscriber("/main_robot/response", String, response_callback)

        rospy.spin()
    except rospy.ROSInterruptException:
        pass
