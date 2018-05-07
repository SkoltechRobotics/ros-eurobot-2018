#! /usr/bin/env python

from executor import *
from bt_builder import BehaviorTreeBuilder
import pickle
import os
import rospkg
import time
import json
from std_msgs.msg import Int32

print(os.getcwd())

# MAIN STRATEGY FOR EUROBOT MOSCOW
SIDE = rospy.get_param("/field/color", "orange")

IS_WASTEWATER_SHOOT = True
HEAPS_ORDER = {
    'orange': '012',
    'green': '543'
}
CUBES_STRATEGY_FILE = "cubes_paths_beta_3.bin"

# MAIN ROBOT STRATEGY
if SIDE == "orange":
    # SIMPLE
    MAIN_ROBOT_STRATEGY = [('start_switch_main', 0), ("heaps", (0, 1)), ("alt_disposal", 0)]
    # MAIN_ROBOT_STRATEGY = [('start_switch_main', 0)]
else:
    # SIMPLE
    # MAIN_ROBOT_STRATEGY = [('bee_main', 0), ("heaps",(5,4)), ("alt_disposal", 0)]
    MAIN_ROBOT_STRATEGY = [('start_switch_main', 0), ("heaps", (5, 4)), ("alt_disposal", 0)]
    # MAIN_ROBOT_STRATEGY = [('start_switch_main', 0)]

# SECOND ROBOT STRATEGY
# SIMPLE
#SMALL_ROBOT_STRATEGY = [("cleanwater_tower_before_waste", 0), ("bee_secondary", 0), ('wastewater_tower', 0)]

# MEDIUM
#SMALL_ROBOT_STRATEGY = [("cleanwater_tower_before_waste", 0), ("bee_secondary", 0), ('wastewater_tower', 0), ('after_wastewater_simple', 0)]

# HARDCORE
SMALL_ROBOT_STRATEGY = [("cleanwater_tower_before_waste", 0), ("bee_secondary", 0), ('cubes_secondary', 0), ('wastewater_tower_after_cubes', 0), ('switch_secondary', 0), ('disposal_secondary', 0)]

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

MAIN_BRAIN = []


class RobotBrain(object):
    def __init__(self, default_name="server", add_str="", server_strategy=None):
        self.name = rospy.get_param("robot_name", default_name)
        self.move_pub = rospy.Publisher(add_str + "move_command", String, queue_size=100)
        self.cmd_pub = rospy.Publisher(add_str + "stm_command", String, queue_size=100)
        self.map_pub = rospy.Publisher("/map_server/cmd", String, queue_size=10)
        self.res_sub = SubscriberHandler(add_str + "response")
        self.pf_cmd_pub = rospy.Publisher(add_str + "pf_cmd", String, queue_size=10)
        self.points_pub = rospy.Publisher(add_str + "point", String, queue_size=100)
        self.statuses_pub = rospy.Publisher(add_str + "action_statuses", String, queue_size=100)

        self.rospack = rospkg.RosPack()

        self.bts = {}
        self.done_bts = []

        self.heap_strats = None
        with open(self.rospack.get_path('eurobot_decision_maker') + "/scripts/" + CUBES_STRATEGY_FILE, "rb") as f:
            self.heap_strats = pickle.load(f)

        self.statuses = []
        # default, about robot
        self.strategy = []
        if self.name == "main_robot":
            self.strategy = MAIN_ROBOT_STRATEGY
        elif self.name == "secondary_robot":
            self.strategy = SMALL_ROBOT_STRATEGY
        self.plan_index = 0
        self.heaps_order = HEAPS_ORDER[SIDE]
        self.new_btb = None
        self.prepare_new_bt(self.strategy, 0, self.heaps_order)
        self.current_btb = self.new_btb
        self.current_bt = self.current_btb.bt
        self.new_btb = None
        self.is_active = False
        self.is_finished = False

        # default, about server
        if server_strategy:
            self.server_strategy = server_strategy
        else:
            if self.name == "main_robot":
                self.server_strategy = [""]

        # points
        self.points = {'secondary_robot': 0, 'main_robot': 0}
        self.sub_points = {name: rospy.Subscriber("/" + name + "/point", String, self.refresh_points(name))
                           for name in self.points}

        self.change_bt_sub = rospy.Subscriber("change_strategy", String, self.change_strategy_callback)
        self.change_plan_sub = rospy.Subscriber("/server/plan", String, self.change_plan_callback)

    def refresh_points(self, whose):
        def cb(msg):
            self.points[whose] = int(msg.data)

        return cb

    def prepare_new_bt(self, new_strategy, new_plan_index, new_heaps_order):
        self.strategy = new_strategy
        self.plan_index = new_plan_index
        self.heaps_order = new_heaps_order
        self.cubes_sequence = self.heap_strats[self.plan_index][self.heaps_order]
        self.new_btb = BehaviorTreeBuilder(self.name, self.move_pub, self.cmd_pub, self.map_pub,
                                           self.res_sub, self.res_sub, move_type='standard')
        self.new_btb.add_strategy(self.strategy)
        self.new_btb.add_cubes_sequence_new(self.cubes_sequence)
        self.new_btb.create_tree_from_strategy(wire_start=False)

    def change_strategy_callback(self, msg):
        msg_splitted = msg.data.split()
        heaps_order = msg_splitted[0]
        strategy = json.loads(reduce(lambda a, b: a + ' ' + b, msg_splitted[1:]))
        self.prepare_new_bt(strategy, self.plan_index, heaps_order)

    def change_plan_callback(self, msg):
        plan_str = msg.data.split()
        new_index = self.plan_index
        if plan_str in POSSIBLE_PLANS:
            new_index = POSSIBLE_PLANS.index(plan_str)
        if plan_str in INV_POSSIBLE_PLANS:
            new_index = INV_POSSIBLE_PLANS.index(plan_str)
        if new_index != self.plan_index and not self.is_active:
            self.prepare_new_bt(self.strategy, new_index, self.heaps_order)

    def start_strategy(self, changed_only=False):
        if not changed_only:
            for i in range(3):
                self.cmd_pub.publish("start_" + self.name + " 164")
                time.sleep(0.05)
        self.is_active = True
        self.current_bt.root_node.start()
        return "finished"

    def stop_strategy(self, is_finished=True):
        self.is_active = False
        if is_finished:
            self.is_finished = True
            self.move_pub.publish("stop_main_cmd stop")
        self.current_bt.root_node.finish()
        self.done_bts.append(self.current_bt)
        return "finished"

    def change_bt(self):
        if self.new_btb:
            if self.is_active:
                self.stop_strategy(False)
                self.current_btb = self.new_btb
                self.current_bt = self.current_btb.bt
                self.new_btb = None
                self.start_strategy(True)
            else:
                self.current_btb = self.new_btb
                self.current_bt = self.current_btb.bt
                self.new_btb = None
        return "finished"

    def fail_until_end(self):
        if self.is_finished:
            return "finished"
        else:
            return "failed"

    def monitor_big_actions(self):
        prev_statuses = self.statuses
        self.statuses = [node.status for node in self.current_bt.root_node.child.children_list]
        statuses_str = reduce(lambda x, y: x + ' ' + y, self.statuses)
        prev_statuses_str = reduce(lambda x, y: x + ' ' + y, prev_statuses)
        if statuses_str != prev_statuses_str:
            self.statuses_pub.publish(statuses_str)

    def calculate_points(self):
        bts = self.done_bts + [self.current_bt]  # + brain_secondary.done_bts
        is_disposal = False
        is_bee = False
        is_button = False
        is_wastewater_tower = False
        is_wastewater_reservoir = False
        is_move_wastewater_tower = False
        is_move_cleanwater_tower = False
        is_cleanwater_tower = False
        is_disposal_secondary = False

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
                    elif child.name.find("switch_main") != -1:
                        is_button = True
                    elif child.name.find("strategy") != -1:
                        ind = child.name.find("strategy")
                        p = int(child.name[ind:].split("_")[1])
                        heap_points = max(heap_points, p)
                    elif child.name.find("wastewater_tower") != -1:
                        is_wastewater_tower = True
                    elif child.name.find("wastewater_reservoir") != -1:
                        is_wastewater_reservoir = True
                    elif child.name.find("disposal_secondary") != -1:
                        is_disposal_secondary = True
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
                 is_button * 25 + \
                 is_move_wastewater_tower * 10 + \
                 is_move_cleanwater_tower * 10
                 is_disposal_secondary * 4
        self.points[self.name] = points
        self.points_pub.publish(str(points))
        global_points_pub.publish(str(sum([v for k, v in self.points.items()])))
        # rospy.loginfo("POINTS " + str(points))
        return "finished"

    def set_robot_server_strategy(self):
        self.bt = BehaviorTree("server_" + self.name)
        general = SequenceNode("general")
        self.bt.add_node(general, self.bt.root_node.name)
        self.bt.add_node(ActionNode("start_plan_recognition", camera_cmd_pub, "start", None, True), "general")
        main_cycle_if = TryUntilSuccessNode("main_cycle_if")
        self.bt.add_node(main_cycle_if, "general")

        # The main cycle if
        main_cycle = SequenceNode("main_cycle")
        main_cycle_if.set_child(main_cycle)
        self.bt.nodes[main_cycle.name] = main_cycle
        self.bt.add_node(TimeoutNode("half_sec_wait", 0.1), "main_cycle")
        self.bt.add_node(ActionFunctionNode("init_robot", self.change_bt), "main_cycle")
        self.bt.add_node(ActionFunctionNode("wait_wire", wait_wire), "main_cycle")

        # The main sequence after start
        self.bt.add_node(ActionNode("stop_plan_recognition", camera_cmd_pub, "finish", None, True), "general")
        self.bt.add_node(ActionFunctionNode("start_main", self.start_strategy), "general")
        self.bt.add_node(ActionNode("stop_wait_wire", stm_node_cmd_pub, "stop_wire", None, True), "general")
        active_work = ParallelNode("active_work")
        self.bt.add_node(active_work, "general")
        wait_and_stop = SequenceNode("wait_and_stop")
        self.bt.add_node(wait_and_stop, "active_work")

        change_bt_cycle_if = TryUntilSuccessNode("change_bt_cycle_if")
        self.bt.add_node(change_bt_cycle_if, "active_work")
        change_bt_cycle = SequenceNode("change_bt_cycle")
        change_bt_cycle_if.set_child(change_bt_cycle)
        self.bt.nodes[change_bt_cycle.name] = change_bt_cycle

        self.bt.add_node(TimeoutNode("little_wait", 0.5), "change_bt_cycle")
        self.bt.add_node(ActionFunctionNode("change_bt_if_needed", self.change_bt), "change_bt_cycle")
        self.bt.add_node(ActionFunctionNode("refresh_points", self.calculate_points), "change_bt_cycle")
        self.bt.add_node(ActionFunctionNode("fail_until_end", self.fail_until_end), "change_bt_cycle")

        self.bt.add_node(TimeoutNode("100_sec_wait", 98.5), "wait_and_stop")
        self.bt.add_node(ActionFunctionNode("stop_main", self.stop_strategy), "wait_and_stop")

    def start_server(self):
        self.bt.root_node.start()


def wire_callback(data):
    global wire_value
    wire_value = int(data.data)


def wait_wire(value=1):
    global wire_value
    if wire_value == value:
        return "finished"
    else:
        return "failed"


if __name__ == "__main__":
    wire_value = 0
    points = 0
    current_plan = ['orange', 'black', 'green']
    rospy.init_node("btb_server_node", anonymous=True)
    rospy.sleep(1.0)
    camera_cmd_pub = rospy.Publisher("/server/camera_command", String, queue_size=100)
    stm_node_cmd_pub = rospy.Publisher("/server/stm_node_command", String, queue_size=10)
    global_points_pub = rospy.Publisher("/server/point", String, queue_size=100)
    res_sub = SubscriberHandler("/server/response")
    rospy.Subscriber("/server/wire_status", String, wire_callback)

    b = RobotBrain()
    #s = RobotBrain("secondary_robot")
    b.set_robot_server_strategy()
    b.start_server()
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        r.sleep()
