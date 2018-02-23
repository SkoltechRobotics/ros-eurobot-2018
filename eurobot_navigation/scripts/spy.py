!#/usr/bin/local python
import rospy


def scan_callback():
    return

if __name__ == "__main__":
    rospy.init_node("spy", anonymous=True)
    rospy.Subscriber("/spy/scan", scan_callback, queue_size = 1)

    rospy.spin()
