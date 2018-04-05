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
    cmd_pub  = rospy.Publisher("/server/command",  String, queue_size=100)
    # map_pub  = rospy.Publisher("/map_server/cmd", String, queue_size=10)
    bt = BehaviorTree("server")
    general = SequenceNode("general")
    bt.add_node(general,"server")
    r = rospy.Rate(10)
    while not rospy.is_shutdown() and bt.root_node.check_status() != "finished":
        r.sleep()
