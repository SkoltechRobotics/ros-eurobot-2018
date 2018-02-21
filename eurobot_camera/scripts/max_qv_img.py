import io
import picamera
import cv2
import rospy
import cv_bridge
from sensor_msgs.msg import Image
import numpy as np

if __name__ == '__main__':
    rospy.init_node('img_publisher', anonymous=True)
    img_pub = rospy.Publisher("image", Image, queue_size=1)

    camera = picamera.PiCamera()
    stream = io.BytesIO()
    bridge = cv_bridge.CvBridge()

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        camera.capture(stream, format="png")
        data = np.fromstring(stream.getvalue(), dtype=np.uint8)
        img = cv2.imdecode(data, 1)
        img_pub.publish(bridge.cv2_to_imgmsg(img, "bgr8"))
        rate.sleep()
