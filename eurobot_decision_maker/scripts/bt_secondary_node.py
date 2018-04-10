#! /usr/bin/env python

from executor import *
# from optimizer import *
import copy
import numpy as np
from bt_builder import BehaviorTreeBuilder
# from cube_picking_optimizer import *


import sys
from strategy_operator import StrategyOperator


class BtSecondary(object):
    def __init__(self):
        rospy.Subscriber("/server/bt_command", String, self.bt_callback)

        self.move_pub = rospy.Publisher("/secondary_robot/move_command", String, queue_size=100)
        self.cmd_pub = rospy.Publisher("/secondary_robot/stm_command", String, queue_size=100)
        self.map_pub = rospy.Publisher("/map_server/cmd", String, queue_size=10)

        self.btb = None

    def bt_callback(self, data):
        pass

    def init_strategy(self):
        self.btb = BehaviorTreeBuilder("secondary_robot", self.move_pub, self.cmd_pub, self.map_pub,
                                       "/secondary_robot/response", "/secondary_robot/response", move_type='standard')
        self.btb.create_tree_from_strategy(wire_start=False)

    def start_strategy(self):
        self.btb.bt.root_node.start()
        r = rospy.Rate(10)
        while not rospy.is_shutdown() and self.btb.bt.root_node.check_status() != "finished":
            r.sleep()

if __name__ == "__main__":
    rospy.init_node("btb_secondary_node", anonymous=True)

    # btb.add_strategy([("heaps",1),("funny",1),("heaps",2),("heaps",0),("disposal",0),("funny",0)])
    # btb.add_strategy([("cleanwater_tower_before_waste",0)])
    btb.add_strategy([("cleanwater_tower_before_waste", 0), ("wastewater_tower", 0)])



