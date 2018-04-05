#! /usr/bin/env python

from executor import *
# from optimizer import *
import copy
import numpy as np
from bt_builder import BehaviorTreeBuilder
#from cube_picking_optimizer import *

                
import sys
from strategy_operator import StrategyOperator

if __name__ == "__main__":
    rospy.init_node("btb_server_node", anonymous=True)
    rospy.sleep(1.0)
    cmd_pub  = rospy.Publisher("/server/command",  String, queue_size=100)
    # map_pub  = rospy.Publisher("/map_server/cmd", String, queue_size=10)
    res_sub = "/server/response"
    bt = BehaviorTree("server")
    general = SequenceNode("general")
    bt.add_node(general,"server")
    plan_start = ActionNode("start plan recognition", cmd_pub, "camera_command start", res_sub)
    plan_stop  = ActionNode("stop_plan_recognition", cmd_pub, "camera_command stop", res_sub)
    main_cycle_if = TryUntilSuccessNode("main_cycle_if")
    main_cycle    = SequenceNode("main_cycle")
    main_cycle_if.set_child(main_cycle)

    r = rospy.Rate(10)
    while not rospy.is_shutdown() and bt.root_node.check_status() != "finished":
        r.sleep()
