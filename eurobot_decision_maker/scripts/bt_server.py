#! /usr/bin/env python

from executor import *
from bt_builder import BehaviorTreeBuilder
import pickle
import os
import rospkg
import time
from std_msgs.msg import Int32

print(os.getcwd())

# MAIN STRATEGY FOR EUROBOT MOSCOW
SIDE = rospy.get_param("/field/color")
IS_WASTEWATER_SHOOT = True
CUBES_ORDER = {
    'orange': '012',
    'green':  '543'
}
# MAIN ROBOT STRATEGY
if SIDE == "orange":
    # SIMPLE
    MAIN_ROBOT_STRATEGY = [('start_switch_main', 0), ("heaps",(0,1)), ("alt_disposal", 0)]
    #MAIN_ROBOT_STRATEGY = [('start_switch_main', 0)]
else:
    # SIMPLE
    #MAIN_ROBOT_STRATEGY = [('bee_main', 0), ("heaps",(5,4)), ("alt_disposal", 0)]
    MAIN_ROBOT_STRATEGY = [('start_switch_main', 0), ("heaps",(5,4)), ("alt_disposal", 0)]
    #MAIN_ROBOT_STRATEGY = [('start_switch_main', 0)]

# SECOND ROBOT STRATEGY
if SIDE == "orange":
    # SIMPLE
    #SMALL_ROBOT_STRATEGY = [("cleanwater_tower_before_waste", 0), ("bee_secondary", 0)]

    # MEDIUM
    SMALL_ROBOT_STRATEGY = [("cleanwater_tower_before_waste", 0), ("bee_secondary", 0), ('wastewater_tower',0)]
else:
    # SIMPLE
    SMALL_ROBOT_STRATEGY = [("cleanwater_tower_before_waste",0), ("bee_secondary", 0)]

    # MEDIUM
    #SMALL_ROBOT_STRATEGY = [("cleanwater_tower_before_waste",0), ("bee_secondary", 0), ('wastewater_tower',0)]

EMERGENCY_MAIN_ROBOT_STRATEGY = [("alt_disposal", 0)]
# EMERGENCY_MAIN_ROBOT_STRATEGY = [("switch_main", 0)]
if SIDE == "orange":
    # EMERGENCY_SECONDARY_ROBOT_STRATEGY = [("switch_secondary", 0)]
    EMERGENCY_SECONDARY_ROBOT_STRATEGY = [("bee_secondary", 0)]
else:
    EMERGENCY_SECONDARY_ROBOT_STRATEGY = [("bee_secondary", 0)]

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

INV_POSSIBLE_PLANS = [x[::-1] for x in POSSIBLE_PLANS]
N_STR = 10


class MainRobotBrain(object):
    def __init__(self):
        rospy.loginfo("INIT MAIN ROBOT STRATEGY ---------------------------")
        self.move_pub = rospy.Publisher("/main_robot/move_command", String, queue_size=100)
        self.cmd_pub = rospy.Publisher("/main_robot/stm_command", String, queue_size=100)
        self.map_pub = rospy.Publisher("/map_server/cmd", String, queue_size=10)
        self.res_sub = SubscriberHandler("/main_robot/response")
        self.pf_cmd_pub = rospy.Publisher("/main_robot/pf_cmd", String, queue_size=10)

        self.rospack = rospkg.RosPack()

        self.bts = {}
        with open(self.rospack.get_path('eurobot_decision_maker') + "/scripts/cubes_paths_beta_3.bin", "rb") as f:
            heap_strats = pickle.load(f)
        for i in range(N_STR):
            btb = BehaviorTreeBuilder("main_robot", self.move_pub, self.cmd_pub, self.map_pub,
                                      self.res_sub, self.res_sub, move_type='standard')
            btb.add_strategy(MAIN_ROBOT_STRATEGY)
            if SIDE == "orange":
                btb.add_cubes_sequence_new(heap_strats[i]['012'])
            else:
                btb.add_cubes_sequence_new(heap_strats[i]['543'])
            btb.create_tree_from_strategy(wire_start=False)
            self.bts[i] = btb.bt
        self.current_bt = self.bts[0]
        self.is_active = False

        btb = BehaviorTreeBuilder("main_robot", self.move_pub, self.cmd_pub, self.map_pub,
                                  self.res_sub, self.res_sub, move_type='standard')
        btb.add_strategy(EMERGENCY_MAIN_ROBOT_STRATEGY)
        btb.create_tree_from_strategy(wire_start=False)
        self.emerge_bt = btb.bt

        self.is_emerge = False

        self.done_bts = []

    def init_strategy(self, plan):
        if plan in POSSIBLE_PLANS:
            #print("USED PLAN ", plan)
            self.current_bt = self.bts[POSSIBLE_PLANS.index(plan)]
        if plan in INV_POSSIBLE_PLANS:
            #print("USED PLAN", plan)
            self.current_bt = self.bts[INV_POSSIBLE_PLANS.index(plan)]
        # btb = BehaviorTreeBuilder("main_robot", self.move_pub, self.cmd_pub, self.map_pub,
        #                           "/main_robot/response", "/main_robot/response", move_type='standard')
        # btb.add_strategy(MAIN_ROBOT_STRATEGY)
        # btb.create_tree_from_strategy(wire_start=False)
        return "finished"

    def start_strategy(self):
        for i in range(3):
            self.cmd_pub.publish("start_main 164")
            time.sleep(0.05)
        # self.pf_cmd_pub.publish("reset")
        self.is_active = True
        self.current_bt.root_node.start()
        return "finished"

    def emergency_strategy(self):
        if not self.is_active:
            return "finished"
        else:
            if self.current_bt.root_node.status == "failed" and not self.is_emerge:
                self.done_bts.append(self.current_bt)
                self.current_bt = self.emerge_bt
                self.current_bt.root_node.start()
                self.is_emerge = True
        return "active"

    def stop_strategy(self):
        self.is_active = False
        self.move_pub.publish("stop_main_cmd stop")
        self.current_bt.root_node.finish()
        self.done_bts.append(self.current_bt)
        return "finished"


class SecondaryRobotBrain(object):
    def __init__(self):
        rospy.loginfo("INIT SECONDARY ROBOT STRATEGY ---------------------------")
        self.move_pub = rospy.Publisher("/secondary_robot/move_command", String, queue_size=100)
        self.cmd_pub = rospy.Publisher("/secondary_robot/stm_command", String, queue_size=100)
        self.map_pub = rospy.Publisher("/map_server/cmd", String, queue_size=10)
        self.res_sub = SubscriberHandler("/secondary_robot/response")
        self.btb = BehaviorTreeBuilder("secondary_robot", self.move_pub, self.cmd_pub, self.map_pub,
                                       self.res_sub, self.res_sub, move_type='standard')
        self.pf_cmd_pub = rospy.Publisher("/secondary_robot/pf_cmd", String, queue_size=10)

        self.btb.add_strategy(SMALL_ROBOT_STRATEGY)
        self.btb.create_tree_from_strategy(wire_start=False)
        self.current_bt = self.btb.bt
        self.is_active = False
        self.done_bts = []

        btb = BehaviorTreeBuilder("main_robot", self.move_pub, self.cmd_pub, self.map_pub,
                                  self.res_sub, self.res_sub, move_type='standard')
        btb.add_strategy(EMERGENCY_SECONDARY_ROBOT_STRATEGY)
        btb.create_tree_from_strategy(wire_start=False)
        self.emerge_bt = btb.bt

        self.is_emerge = False

    def init_strategy(self):
        return "finished"

    def start_strategy(self):
        for i in range(3):
            self.cmd_pub.publish("start_secondary 164")
            time.sleep(0.05)
        # self.pf_cmd_pub.publish("reset")
        self.is_active = True
        self.current_bt.root_node.start()
        return "finished"

    def emergency_strategy(self):
        if not self.is_active:
            return "finished"
        else:
            if self.current_bt.root_node.status in ["failed", "error"] and not self.is_emerge:
                self.done_bts.append(self.current_bt)
                self.current_bt = self.emerge_bt
                self.current_bt.root_node.start()
                self.is_emerge = True
        return "active"

    def stop_strategy(self):
        self.is_active = False
        self.move_pub.publish("stop_secondary_cmd stop")
        self.current_bt.root_node.finish()
        self.done_bts.append(self.current_bt)
        return "finished"


def calculate_points():
    global points
    bts = brain_main.done_bts + brain_secondary.done_bts

    is_disposal = False
    is_bee = False
    is_button = False
    is_wastewater_tower = False
    is_wastewater_reservoir = False
    is_move_wastewater_tower = False
    is_move_cleanwater_tower = False
    is_cleanwater_tower = False

    heap_points = 0
    balls = 0
    for bt1 in bts:
        for child in bt1.root_node.child.children_list:
            print(child.name)
            print(child.status)
            if child.status == "finished":
                if child.name.find("parallel_shift_down_man") != -1:
                    is_disposal = True
                elif child.name.find("bee") != -1:
                    is_bee = True
                elif child.name.find("switch") != -1:
                    is_button = True
                elif child.name.find("strategy") != -1:
                    ind = child.name.find("strategy")
                    p = int(child.name[ind:].split("_")[1])
                    heap_points = max(heap_points, p)
                elif child.name.find("wastewater_tower") != -1:
                    is_wastewater_tower = True
                elif child.name.find("wastewater_reservoir") != -1:
                    is_wastewater_reservoir = True
            if child.name.find("wastewater_tower") != -1:
                for child1 in child.children_list:
                    if child1.name.find("move") != -1 and child1.status == "finished":
                        is_move_wastewater_tower = True
            if child.name.find("cleanwater_tower") != -1:
                if child.status == "finished":
                    is_cleanwater_tower = True
                for child1 in child.children_list:
                    if child1.name.find("sort_and_shoot") != -1:
                        print(child1.name)
                        print(child1.status)
                        if child1.status == "finished":
                            balls += 1
                    if child1.name.find("move") != -1:
                        print(child1.name)
                        print(child1.status)
                        if child1.status == "finished":
                            is_move_cleanwater_tower = True
    is_wastewater_reservoir = IS_WASTEWATER_SHOOT
    points = 10 + \
        is_disposal * heap_points + \
        is_cleanwater_tower * 40 + \
        is_wastewater_tower * is_wastewater_reservoir * 30 + \
        is_bee * 50 + \
        is_button * 25 +\
        is_move_wastewater_tower * 10 +\
        is_move_cleanwater_tower * 10
    points_pub.publish(str(points))
    rospy.loginfo("POINTS " + str(points))
    return "finished"


def wire_callback(data):
    global wire_value
    wire_value = int(data.data)


def plan_callback(data):
    global current_plan
    current_plan = data.data.split()


def wait_wire(value):
    global wire_value
    if wire_value == value:
        return "finished"
    else:
        return "failed"


def init_main_robot_from_plan():
    global current_plan
    global brain_main
    return brain_main.init_strategy(current_plan)


if __name__ == "__main__":
    wire_value = 0
    points = 0
    current_plan = ['orange', 'black', 'green']
    rospy.init_node("btb_server_node", anonymous=True)
    rospy.sleep(1.0)
    camera_cmd_pub = rospy.Publisher("/server/camera_command", String, queue_size=100)
    stm_node_cmd_pub = rospy.Publisher("/server/stm_node_command", String, queue_size=10)
    points_pub = rospy.Publisher("/server/point", String, queue_size=100)
    res_sub = SubscriberHandler("/server/response")
    rospy.Subscriber("/server/plan", String, plan_callback)
    rospy.Subscriber("/server/wire_status", String, wire_callback)
    brain_secondary = SecondaryRobotBrain()
    brain_main = MainRobotBrain()
    # BT root
    bt = BehaviorTree("server")
    general = SequenceNode("general")
    bt.add_node(general, "server")

    # The main sequence before start
    bt.add_node(ActionNode("start_plan_recognition", camera_cmd_pub, "start", res_sub, True), "general")
    bt.add_node(ActionFunctionNode("init_secondary", brain_secondary.init_strategy), "general")
    main_cycle_if = TryUntilSuccessNode("main_cycle_if")
    bt.add_node(main_cycle_if, "general")

    # The main cycle if
    main_cycle = SequenceNode("main_cycle")
    main_cycle_if.set_child(main_cycle)
    bt.nodes[main_cycle.name] = main_cycle
    bt.add_node(ActionFunctionNode("init_main_robot_from_plan", init_main_robot_from_plan), "main_cycle")
    bt.add_node(ActionFunctionNode("wait_wire_1", lambda: "finished" if not wait_wire(1) else "error"), "main_cycle")
    bt.add_node(TimeoutNode("half_sec_wait", 0.5), "main_cycle")

    # The main sequence after start
    bt.add_node(ActionNode("stop_plan_recognition", camera_cmd_pub, "finish", res_sub, True), "general")
    bt.add_node(ActionFunctionNode("start_main", brain_main.start_strategy), "general")
    bt.add_node(ActionFunctionNode("start_secondary", brain_secondary.start_strategy), "general")
    bt.add_node(ActionNode("stop_wait_wire", stm_node_cmd_pub, "stop_wire", res_sub, True), "general")

    # # The stop
    active_work = ParallelNode("active_work")
    bt.add_node(active_work, "general")
    wait_and_stop = SequenceNode("wait_and_stop")
    bt.add_node(wait_and_stop, "active_work")

    bt.add_node(TimeoutNode("100_sec_wait", 98.5), "wait_and_stop")
    bt.add_node(ActionFunctionNode("recover_main_robot", brain_main.emergency_strategy), "active_work")
    bt.add_node(ActionFunctionNode("recover_secondary_robot", brain_secondary.emergency_strategy), "active_work")
    bt.add_node(ActionFunctionNode("stop_main", brain_main.stop_strategy), "wait_and_stop")
    bt.add_node(ActionFunctionNode("stop_secondary", brain_secondary.stop_strategy), "wait_and_stop")
    bt.add_node(ActionFunctionNode("calculate_points", calculate_points), "general")
    r = rospy.Rate(10)
    bt.root_node.start()
    while not rospy.is_shutdown() and bt.root_node.check_status() != "finished":
        r.sleep()
