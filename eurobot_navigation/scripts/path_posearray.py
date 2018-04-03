#!/usr/bin/env python
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseArray


def callback(path):
    poses = [pose.pose for pose in path.poses]
    pa = PoseArray(poses = poses)
    pa.header.frame_id = "map"
    pub.publish(pa)

if __name__ == "__main__":
    rospy.init_node("path_posearray_publisher")
    rospy.Subscriber("global_planner/planner/plan", Path, callback)
    pub = rospy.Publisher("path_posearray", PoseArray, queue_size=1)
    rospy.spin()
