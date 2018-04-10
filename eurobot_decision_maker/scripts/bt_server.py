#! /usr/bin/env python

from executor import *
from bt_builder import BehaviorTreeBuilder
import pickle
import os
print(os.getcwd())

SMALL_ROBOT_STRATEGY = [("wastewater_tower", 0), ("wastewater_reservoir", 0), ("cleanwater_tower_after_waste", 0)]
MAIN_ROBOT_STRATEGY = [("heaps", 0), ("heaps", 1), ("heaps", 2), ("disposal", 0)]

POSSIBLE_PLANS = [
    ['orange', 'black', 'green'],
    ['blue', 'black', 'yellow'],
    ['blue', 'green', 'orange'],
    ['yellow', 'green', 'black'],
    ['black', 'yellow', 'orange'],
    ['green', 'yellow', 'blue'],
    ['blue', 'orange', 'black'],
    ['green', 'orange', 'yellow'],
    ['black', 'blue', 'green'],
    ['orange', 'blue', 'yellow']
]
N_STR = 10


class BtMain(object):
    def __init__(self):
        rospy.loginfo("INIT MAIN ROBOT STRATEGY ---------------------------")
        self.move_pub = rospy.Publisher("/main_robot/move_command", String, queue_size=100)
        self.cmd_pub = rospy.Publisher("/main_robot/stm_command", String, queue_size=100)
        self.map_pub = rospy.Publisher("/map_server/cmd", String, queue_size=10)

        self.bts = {}
        with open("very_important_bt_paths2.bin", "rb") as f:
            heap_strats = pickle.load(f)
        for i in range(N_STR):
            btb = BehaviorTreeBuilder("main_robot", self.move_pub, self.cmd_pub, self.map_pub,
                                      "/main_robot/response", "/main_robot/response", move_type='standard')
            btb.add_strategy(MAIN_ROBOT_STRATEGY)
            btb.add_cubes_sequence_new(heap_strats[i]['001'])
            btb.create_tree_from_strategy(wire_start=False)
            self.bts[i] = btb.bt
        self.bt = self.bts[0]

    def init_strategy(self, plan):
        self.bt = self.bts[POSSIBLE_PLANS.index(plan)]
        return 0

    def start_strategy(self):
        self.bt.root_node.start()
        return 0

    def stop_strategy(self):
        self.bt.root_node.finish()
        return 0


class BtSecondary(object):
    def __init__(self):
        rospy.loginfo("INIT SECONDARY ROBOT STRATEGY ---------------------------")
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


def plan_callback(data):
    global current_plan
    current_plan = data.data.split()


def wait_wire(value):
    global wire_value
    if wire_value == value:
        return 0
    else:
        return 1


def init_main_robot_from_plan():
    global current_plan
    global bt_main
    return bt_main.init_strategy(current_plan)


if __name__ == "__main__":
    wire_value = 1
    current_plan = ['orange', 'black', 'green']
    rospy.init_node("btb_server_node", anonymous=True)
    rospy.sleep(1.0)
    camera_cmd_pub = rospy.Publisher("/server/camera_command", String, queue_size=100)
    stm_node_cmd_pub = rospy.Publisher("/server/stm_node_command", String, queue_size=10)
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
    bt.add_node(ActionNode("start_wait_wire", stm_node_cmd_pub, "start_wire", res_sub, True), "general")
    bt.add_node(ActionFunctionNode("wait_wire_0", lambda: wait_wire(0)), "general")
    bt.add_node(ActionNode("start_plan_recognition", camera_cmd_pub, "start", res_sub, True), "general")
    bt.add_node(ActionFunctionNode("init_secondary", bt_secondary.init_strategy), "general")
    main_cycle_if = TryUntilSuccessNode("main_cycle_if")
    bt.add_node(main_cycle_if, "general")

    # The main cycle if
    main_cycle = SequenceNode("main_cycle")
    main_cycle_if.set_child(main_cycle)
    bt.nodes[main_cycle.name] = main_cycle
    bt.add_node(ActionFunctionNode("init_main_robot_from_plan", init_main_robot_from_plan), "main_cycle")
    bt.add_node(ActionFunctionNode("wait_wire_1", lambda: 0 if not wait_wire(1) else 2), "main_cycle")
    bt.add_node(TimeoutNode("half_sec_wait", 0.5), "main_cycle")

    # The main sequence after start
    bt.add_node(ActionNode("stop_plan_recognition", camera_cmd_pub, "finish", res_sub, True), "general")
    bt.add_node(ActionFunctionNode("start_main", bt_main.start_strategy), "general")
    bt.add_node(ActionFunctionNode("start_secondary", bt_secondary.start_strategy), "general")
    bt.add_node(ActionNode("stop_wait_wire", stm_node_cmd_pub, "stop_wire", res_sub, True), "general")

    # # The stop
    bt.add_node(TimeoutNode("100_sec_wait", 100), "general")
    bt.add_node(ActionFunctionNode("stop_main", bt_main.stop_strategy), "general")
    bt.add_node(ActionFunctionNode("stop_secondary", bt_secondary.stop_strategy), "general")
    r = rospy.Rate(10)
    bt.root_node.start()
    while not rospy.is_shutdown() and bt.root_node.check_status() != "finished":
        r.sleep()
