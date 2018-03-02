#!/usr/bin/env python
import rospy
from move_base_msgs.msg import MoveBaseActionGoal
import sys

if __name__ == '__main__':
    rospy.init_node("test_goal_pub")
    pub = rospy.Publisher("/move_base/goal", MoveBaseActionGoal, queue_size=3)

    x, y = int(sys.argv[1]), int(sys.argv[2])

    move_message = MoveBaseActionGoal()
    move_message.goal_id.id = sys.argv[3]
    move_message.goal.target_pose.header.frame_id = 'world'

    move_message.goal.target_pose.pose.position.x = x
    move_message.goal.target_pose.pose.position.y = y

    move_message.goal.target_pose.pose.orientation.x = 0
    move_message.goal.target_pose.pose.orientation.y = 0
    move_message.goal.target_pose.pose.orientation.z = 0
    move_message.goal.target_pose.pose.orientation.w = 1
    rospy.sleep(0.2)
    print(move_message)
    pub.publish(move_message)
    rospy.sleep(0.5)
