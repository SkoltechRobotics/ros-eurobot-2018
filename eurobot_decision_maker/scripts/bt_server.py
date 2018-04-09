#! /usr/bin/env python

from executor import *
# from optimizer import *
import copy
import numpy as np
from bt_builder import BehaviorTreeBuilder
# from cube_picking_optimizer import *
import sys
from strategy_operator import StrategyOperator
import pickle


SMALL_ROBOT_STRATEGY = [("wastewater_tower", 0), ("wastewater_reservoir", 0), ("cleanwater_tower_after_waste", 0)]


class BtMain(object):
    def __init__(self):
        self.move_pub = rospy.Publisher("/main_robot/move_command", String, queue_size=100)
        self.cmd_pub = rospy.Publisher("/main_robot/stm_command", String, queue_size=100)
        self.map_pub = rospy.Publisher("/map_server/cmd", String, queue_size=10)
        self.btb = BehaviorTreeBuilder("secondary_robot", self.move_pub, self.cmd_pub, self.map_pub,
                                       "/secondary_robot/response", "/secondary_robot/response", move_type='standard')
        self.btb.create_tree_from_strategy(wire_start=False)
        with open("very_important_bt_paths2.bin", "rb") as f:
            heap_strats = pickle.load(f)
        self.btb.add_cubes_sequence_new(heap_strats[0]['001'])
        self.bt = self.btb.bt

    def init_strategy(self, plan):
        return 0

    def start_strategy(self):
        self.bt.root_node.start()
        return 0

    def stop_strategy(self):
        self.bt.root_node.finish()
        return 0


class BtSecondary(object):
    def __init__(self):
        self.move_pub = rospy.Publisher("/secondary_robot/move_command", String, queue_size=100)
        self.cmd_pub = rospy.Publisher("/secondary_robot/stm_command", String, queue_size=100)
        self.map_pub = rospy.Publisher("/map_server/cmd", String, queue_size=10)

        self.btb = BehaviorTreeBuilder("secondary_robot", self.move_pub, self.cmd_pub, self.map_pub,
                                       "/secondary_robot/response", "/secondary_robot/response", move_type='standard')
        self.btb.add_strategy(SMALL_ROBOT_STRATEGY)
        self.btb.create_tree_from_strategy(wire_start=False)
        self.bt = self.btb.bt

    def init_strategy(self):
        return 0

    def start_strategy(self):
        self.bt.root_node.start()
        return 0

    def stop_strategy(self):
        self.bt.root_node.finish()
        return 0


def wire_callback(data):
    global wire_value
    wire_value = int(data.data)


def stop_wait_wire():
    pass


def plan_callback(data):
    global plan
    plan = data.data.split()


def wait_wire(value):
    global wire_value
    if wire_value == value:
        return 0
    else:
        return 1


def init_main_robot_from_plan():
    global plan
    global bt_main
    bt_main.init_strategy(plan)


if __name__ == "__main__":
    wire_value = 0
    plan = ["black", "blue", "orange"]
    rospy.init_node("btb_server_node", anonymous=True)
    rospy.sleep(1.0)
    cmd_pub = rospy.Publisher("/server/command", String, queue_size=100)
    res_sub = "/server/response"
    rospy.Subscriber("/server/plan", String, plan_callback)
    rospy.Subscriber("/server/wire_status", String, wire_callback)
    bt_secondary = BtSecondary()
    bt_main = BtMain()
    # BT root
    bt = BehaviorTree("server")
    general = SequenceNode("general")
    bt.add_node(general, "server")

    # The main sequence before start
    bt.add_node(ActionFunctionNode("wait_wire_0", lambda: wait_wire(0)), "general")
    bt.add_node(ActionNode("start plan recognition", cmd_pub, "camera_command start", res_sub), "general")
    bt.add_node(ActionFunctionNode("init_secondary", bt_secondary.init_strategy), "general")
    main_cycle_if = TryUntilSuccessNode("main_cycle_if")
    bt.add_node(main_cycle_if, "general")

    # The main cycle if
    main_cycle = SequenceNode("main_cycle")
    main_cycle_if.set_child(main_cycle)
    bt.add_node(ActionFunctionNode("init_main_robot_from_plan", init_main_robot_from_plan), "main_cycle")
    bt.add_node(ActionFunctionNode("wait_wire_1", lambda: 0 if not wait_wire(1) else 2), "main_cycle")

    # The main sequence after start
    bt.add_node(ActionNode("stop_plan_recognition", cmd_pub, "camera_command stop", res_sub), "general")
    bt.add_node(ActionFunctionNode("start_main", bt_main.start_strategy), "general")
    bt.add_node(ActionFunctionNode("start_secondary", bt_secondary.start_strategy), "general")
    bt.add_node(ActionFunctionNode("stop_wait_wire", stop_wait_wire), "general")

    # # The stop
    bt.add_node(TimeoutNode("100_sec_wait", 100), "general")
    bt.add_node(ActionFunctionNode("stop_main", bt_main.stop_strategy), "general")
    bt.add_node(ActionFunctionNode("stop_secondary", bt_secondary.stop_strategy), "general")
    r = rospy.Rate(10)
    bt.root_node.start()
    while not rospy.is_shutdown() and bt.root_node.check_status() != "finished":
        r.sleep()
