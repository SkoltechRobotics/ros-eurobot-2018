#! /usr/bin/env python

# import sys
# sys.path.append("~/catkin_ws/src/ros-eurobot-2018/eurobot_decision_maker/src/python/executor")
import rospy

from executor import *
from move_base_msgs.msg import MoveBaseActionGoal
from tf.transformations import euler_from_quaternion, quaternion_from_euler


def move_message(x, y, a):
    move_message = MoveBaseActionGoal()

    # edit move_message here

    move_message.goal.target_pose.header.frame_id = 'world'

    move_message.goal.target_pose.pose.position.x = x
    move_message.goal.target_pose.pose.position.y = y

    move_message.goal.target_pose.pose.orientation.x = 0
    move_message.goal.target_pose.pose.orientation.y = 0
    move_message.goal.target_pose.pose.orientation.z = 0
    move_message.goal.target_pose.pose.orientation.w = 0

    o = move_message.goal.target_pose.pose.orientation

    o.x, o.y, o.z, o.w = quaternion_from_euler(*[0, 0, a])

    return move_message


if __name__ == '__main__':
    rospy.init_node("executor", anonymous=True)

    sub_response_name = "/main_robot/response"

    # stm_move parameters
    distance_to_move = 0.058 * 2  # m
    speed_to_move = 0.2  # m/s

    pub_move = rospy.Publisher("/move_base/goal", MoveBaseActionGoal, queue_size=100)
    pub_cmd = rospy.Publisher("/main_robot/stm_command", String, queue_size=100)
    pub_rf = rospy.Publisher("/main_robot/move_command", String, queue_size=100)

    all_actions = SequenceNode("all")
    
    move_to_heap = SequenceNode("move_to_heap")

    nav_move = ActionNode("nav_move", pub_move, move_message(0.85, 0.54 - 0.058 * 3, 0), sub_response_name)
    stm_move = ActionNode("stm_move", pub_cmd, "162 0 " + str(distance_to_move) + " 0 0 " + str(speed_to_move) + " 0",
                          sub_response_name)
    
    
    pick_1 = SequenceNode("pick_1")
    rf_move = ActionNode("rf_move", pub_rf, "MOVETOHEAP", sub_response_name)
    pick_1c = ActionNode("p1c", pub_cmd, "176 1", sub_response_name)
    
    pick_2 = SequenceNode("pick_2")
    radius = 0.06 #m
    speed  = 0.05  #m/s
    angle  = 3.14 #m/s
    stm_rot_pi = ActionNode("stm_move2", pub_cmd, "162 " + str(radius*angle)  + " 0 " + str(angle) + " " + str(speed) + " 0 " + str(speed/radius), sub_response_name)
    rf_move2 = ActionNode("rf_move2", pub_rf, "MOVETOHEAP", sub_response_name)
    
    pick_012 = ParallelNode("pick_012")
    pick_0t = ActionNode("p0t", pub_cmd, "176 0", sub_response_name)
    pick_1t = ActionNode("p1t", pub_cmd, "176 1", sub_response_name)
    pick_2t = ActionNode("p2t", pub_cmd, "176 2", sub_response_name)
    
    pick_3 = SequenceNode("pick_3")
    dist_x = 0.06 #m
    speed_x = 0.2 #m/s
    stm_move3 = ActionNode("stm_move3", pub_cmd, "162 " + str(dist_x) + " 0 0 " + str(speed_x) + " 0 0", sub_response_name)
    pick_last = ActionNode("p2l", pub_cmd, "176 2", sub_response_name)

    rospy.sleep(0.5)

    bt = BehaviorTree("big_robot", 100)
#
    bt.add_node(all_actions, "big_robot")
#
    # bt.add_node(move_to_heap, all_actions.name)
    #
    # bt.add_node(nav_move, move_to_heap.name)
    # bt.add_node(stm_move, move_to_heap.name)
    # bt.add_node(rf_move, move_to_heap.name)
#    
    bt.add_node(pick_1, all_actions.name)

    # bt.add_node(rf_move, pick_1.name)
    bt.add_node(pick_1c, pick_1.name)

#
    bt.add_node(pick_2, all_actions.name)

    bt.add_node(stm_rot_pi, pick_2.name)
    # bt.add_node(rf_move2,   pick_2.name)

    bt.add_node(pick_012, pick_2.name)
    bt.add_node(pick_0t, pick_012.name)
    bt.add_node(pick_1t, pick_012.name)
    bt.add_node(pick_2t, pick_012.name)
#
    bt.add_node(pick_3, all_actions.name)

    bt.add_node(stm_move3, pick_3.name)
    bt.add_node(pick_last, pick_3.name)


    bt.root_node.start()

    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        r.sleep()
