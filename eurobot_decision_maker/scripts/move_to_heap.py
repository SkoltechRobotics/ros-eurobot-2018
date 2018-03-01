#! /usr/bin/env python

#import sys
#sys.path.append("~/catkin_ws/src/ros-eurobot-2018/eurobot_decision_maker/src/python/executor")
import rospy

from executor import *
from move_base_msgs.msg import MoveBaseActionGoal
from tf.transformations import euler_from_quaternion, quaternion_from_euler

if __name__  == '__main__':
    rospy.init_node("executor", anonymous=True)
    
    move_message = MoveBaseActionGoal()

    #edit move_message here


    move_message.goal.target_pose.pose.position.x = 0.85
    move_message.goal.target_pose.pose.position.y = 0.54 - 0.058 - 0.058
    
    move_message.goal.target_pose.pose.orientation.x = 0
    move_message.goal.target_pose.pose.orientation.y = 0
    move_message.goal.target_pose.pose.orientation.z = 0
    move_message.goal.target_pose.pose.orientation.w = 0
    
    print(quaternion_from_euler(*[0,0,4.71]))

    sub_response_name = "/main_robot/response"

    #stm_move parameters
    distance_to_move = 0.06 #m
    speed_to_move    = 0.2  #m/s
    
    pub_move = rospy.Publisher("/move_base/goal", MoveBaseActionGoal, queue_size=100)
    pub_cmd  = rospy.Publisher("/main_robot/stm_command", String, queue_size=100)
    pub_rf   = rospy.Publisher("/main_robot/move_command", String, queue_size=100)

    move_to_heap = SequenceNode("move_to_heap")

    nav_move = ActionNode("nav_move", pub_rf, "MOVE 0.85 0.424 4.71", sub_response_name)
    stm_move = ActionNode("stm_move", pub_cmd,  "162 0 " + str(distance_to_move) + " 0 0 " + str(speed_to_move) + " 0", sub_response_name)
    rf_move  = ActionNode("rf_move" , pub_rf,   "MOVETOHEAP", sub_response_name)
    
    rospy.sleep(0.2)

    bt = BehaviorTree("big_robot", 100)
    
    bt.add_node(move_to_heap, "big_robot")

    bt.add_node(nav_move, move_to_heap.name)
    bt.add_node(stm_move, move_to_heap.name)
    bt.add_node(rf_move,  move_to_heap.name)

    bt.root_node.start()
    
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        r.sleep()
