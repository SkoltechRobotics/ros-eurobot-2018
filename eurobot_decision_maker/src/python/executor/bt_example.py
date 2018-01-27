#! /usr/bin/env python

#import sys
#sys.path.append("~/catkin_ws/src/ros-eurobot-2018/eurobot_decision_maker/src/python/executor")


from executor import *


if __name__  == '__main__':
    rospy.init_node("executor", anonymous=True)

    pub_move = rospy.Publisher("/main_robot/move_command", String, queue_size=100)
    pub_cmd = rospy.Publisher("/main_robot/stm_command", String, queue_size=100)
    a1 = ActionNode("move1", pub_move, "MOVE 500 300 0", "/main_robot/response")
    a2 = ActionNode("move2", pub_move, "MOVE 850 350 0", "/main_robot/response")
    a3 = ActionNode("move3", pub_move, "MOVE 850 482 0", "/main_robot/response")
    a4 = ActionNode("take1", pub_cmd, "176 0", "/main_robot/response")
    a5 = ActionNode("take2", pub_cmd, "176 1", "/main_robot/response")
    a6 = ActionNode("take3", pub_cmd, "176 2", "/main_robot/response")
    a7 = ActionNode("move4", pub_move, "MOVE 850 540 0", "/main_robot/response")
    a8 = ActionNode("take4", pub_cmd, "176 1", "/main_robot/response")
    a9 = ActionNode("move5", pub_move, "MOVE 850 600 0", "/main_robot/response")
    a10 = ActionNode("take5", pub_cmd, "176 1", "/main_robot/response")

    s0 = SequenceNode("s0")
    s1 = SequenceNode("s1")
    p2 = ParallelNode("p2")
    s3 = SequenceNode("s3")
    rospy.sleep(0.2)
    
    
    bt = BehaviorTree("big_robot", 100)

    bt.add_node(s0, "big_robot")

    bt.add_node(s1, s0.name)
    bt.add_node(a1, s1.name)
    bt.add_node(a2, s1.name)
    bt.add_node(a3, s1.name)

    bt.add_node(p2, s0.name)
    bt.add_node(a4, p2.name)
    bt.add_node(a5, p2.name)
    bt.add_node(a6, p2.name)

    bt.add_node(s3, s0.name)
    bt.add_node(a7, s3.name)
    bt.add_node(a8, s3.name)
    bt.add_node(a9, s3.name)
    bt.add_node(a10, s3.name)

    bt.root_node.start()
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        r.sleep()
 
