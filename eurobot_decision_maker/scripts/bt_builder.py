#! /usr/bin/env python

from executor import *
# from optimizer import *
import copy
import numpy as np
import pickle
import rospkg

# from cube_picking_optimizer import *

def rot_matrix(rot_angle):
    return np.matrix([[np.cos(rot_angle), -np.sin(rot_angle), 0],
                      [np.sin(rot_angle), np.cos(rot_angle), 0],
                      [0, 0, 1]])


class BehaviorTreeBuilder:
    scale = {"m": 1000.0, "dm": 100.0, "cm": 10.0, "mm": 1.0}
    com = {
        "pick": {'name': str(176)},
        "pick_last_cube": {'name': str(0xb5)},
        "magic_cube": {'name': str(0xb4), 'left': 0, 'right': 1},
        "funny_action": {'name': str(0xb6), 'open': 1, 'close': 0},
        "bot_sort": {'name': str(0xc3), 'left': 0, 'right': 1, 'mid': 2},
        "up_sort": {'name': str(0xc2), 'clean': 0, 'waste': 1, 'mid': 2},
        "waste_door": {'name': str(0xc1), 'open': 1, 'close': 0},
        'wire_start': {'name': str(0xa4)}
    }
    action_places_both_sides = { 'orange': {
            "heaps": np.array([[54, 85, 0], [119, 30, 0], [150, 110, 0], [150, 190, 0], [119, 270, 0], [54, 215, 0]],
                              dtype=np.float64),
            "funny": np.array([[10, 113, 0], [190, 10, 0]], dtype=np.float64),
            "disposal": np.array([[10, 61, 0]], dtype=np.float64),
            "base": np.array([[15, 15, 0]], dtype=np.float64),
            "wastewater_tower": np.array([[0, 0, 0]], dtype=np.float64),
            "wastewater_reservoir": np.array([[100, 157, 4.71]], dtype=np.float64),
            "cleanwater_tower": np.array([[82.6, 16.6, 4.71]], dtype=np.float64)
        },
        'green' : {
            "heaps": np.array([[54, 85, 0], [119, 30, 0], [150, 110, 0], [150, 190, 0], [119, 270, 0], [54, 215, 0]],
                              dtype=np.float64),
            "funny": np.array([[10, 113, 0], [190, 10, 0]], dtype=np.float64),
            "disposal": np.array([[10, 61, 0]], dtype=np.float64),
            "base": np.array([[15, 15, 0]], dtype=np.float64),
            "wastewater_tower": np.array([[0, 0, 0]], dtype=np.float64),
            "wastewater_reservoir": np.array([[100, 157, 4.71]], dtype=np.float64),
            "cleanwater_tower": np.array([[82.6, 16.6, 4.71]], dtype=np.float64)
        }
    }
    heap_sides = {
        0: (3, 2, 2),
        1: (1, 0, 2),
        2: (0, 0, 1),
        3: (0, 0, 3),
        4: (3, 0, 2),
        5: (1, 2, 2)
    }
    shifts = [(-1, 0), (0, -1), (1, 0), (0, 1)]
    def rotate(self, shift, a):
        alpha = np.pi/2*(a-1)
        m = np.array([[np.cos(alpha), np.sin(alpha)], [-np.sin(alpha), np.cos(alpha)]])
        return m.dot(np.array(shift))


    def __init__(self, bt_name, move_pub, cmd_pub, map_pub, move_response, cmd_response, **kvargs):
        self.id = 0
        self.bt = BehaviorTree(bt_name)
        self.root_seq_name = 'strategy'
        self.bt.add_publisher("move_publisher", move_pub)
        self.bt.add_publisher("cmd_publisher", cmd_pub)
        self.bt.add_publisher("map_publisher", map_pub)
        self.move_response = move_response.topic_name
        self.cmd_response = cmd_response.topic_name
        self.bt.add_subscriber(move_response.topic_name, move_response)
        self.bt.add_subscriber(cmd_response.topic_name, cmd_response)
        self.black_angle = 0  # angle for black cube to be picked by central manipulator
        self.opt_units = "cm"  # from self.opt_units to self.track_units
        self.track_units = "m"
        self.move_action_name = str(0x0E)
        self.move_publisher_name = "cmd_publisher"
        self.side = rospy.get_param("/field/color", "orange")
        self.action_places = self.action_places_both_sides[self.side]

        # main robot
        self.pick_action_name = str(176)
        self.last_cube_pick_action_name = str(0xb5)
        self.magic_cube_action_name = str(0xb4)
        self.funny_action_name = str(0xb6)

        self.heap_score = 0

        self.last_coordinates = [0,0,0]
        self.last_angle = self.black_angle
        self.heaps_sequences = []
        self.strategy_sequence = []
        self.cube_vector = np.array([[0], [5.8], [0.0]])
        self.pick_one_by_one = False

        # secondary_robot
        self.bottom_sorter = str(0xc3)
        self.upper_sorter = str(0xc2)
        self.wastewater_door = str(0xc1)
        self.wt_y_shift = np.array([0, 15, 0], dtype=np.float64)
        self.wt_x_shift = np.array([15, 0, 0], dtype=np.float64)
        self.shooting_motor = str(0xc4)
        self.shoot_poses = {"interm" : 2, "left" : 0, "right" : 1, "release right" : 4, "release left" : 3}
        self.first_poses = {"clean" : 0, "waste" : 1, "interm" : 2, "interm clean" : 3, "interm waste" : 4}

        if 'move_type' in kvargs:
            if kvargs['move_type'] == 'standard':
                self.track_units = "m"
                self.move_action_name = str('move')
                self.move_publisher_name = "move_publisher"
            elif kvargs['move_type'] == 'simple':
                self.track_units = 'm'
                self.move_action_name = str(0x0E)
                self.move_publisher_name = "cmd_publisher"
        # self.action_places = copy.deepcopy(StrategyOptimizer.action_places)
        for _, action in self.action_places.items():
            for coords in action:
                coords[:2] = coords[:2][::-1]

        self.colors_left = {0, 1, 2, 3, 4}
        self.man_load = [0, 0, 0]

    def get_next_id(self):
        self.id += 1
        return self.id

    def construct_string(self, *args, **kvargs):
        sep = "_"
        if "sep" in kvargs:
            sep = kvargs["sep"]
        node_name = ""
        for a in args[:-1]:
            node_name += str(a) + sep
        return node_name + str(args[-1])

    def add_action_node(self, parent_name, prefix, str_pub, str_response, *args):
        node_name = self.construct_string(prefix, self.get_next_id())
        node_description = self.construct_string(parent_name, 'action', node_name, str_pub, str_response, *args,
                                                 sep=' ')
        rospy.loginfo(node_description)
        self.bt.add_node_by_string(node_description)

    def convert_units(self, dist):
        rospy.loginfo(dist)
        return BehaviorTreeBuilder.scale[self.opt_units] / BehaviorTreeBuilder.scale[self.track_units] * dist

    def add_move_action(self, parent_name, *args, **kvargs):
        rospy.loginfo(args)
        move_type = "move"
        if "move_type" in kvargs and kvargs["move_type"] in ["move", "move_odometry", "move_stm"]:
            move_type = kvargs["move_type"]
        shift_multiplier = 1
        if "shift_multiplier" in kvargs:
            shift_multiplier = kvargs["shift_multiplier"]
        args = list(args)
        robot_angle = args[-1]
        if move_type == "move_stm":
            stm_vel = 0.15
            shift_since_last = np.array(args).reshape(3, 1) - np.array(self.last_coordinates).reshape(3, 1)
            for i in range(2):
                shift_since_last[i] = self.convert_units(shift_since_last[i])
            # shift_since_last[:2] = [self.convert_units(a) for a in args[:2]]
            shift_since_last = list(shift_since_last.ravel())
            sh = shift_since_last
            self.add_command_action(parent_name, 162, sh[0], sh[1], 0, 0.15, 0.15, 0)
        self.last_coordinates = copy.deepcopy(args)
        rospy.loginfo(args)
        args[:2] = [self.convert_units(a) for a in args[:2]]
        args = np.array(args).reshape(3, 1)
        rospy.loginfo(args)
        shift_center = rot_matrix(robot_angle).dot(self.convert_units(self.cube_vector * shift_multiplier))
        rospy.loginfo(shift_center)
        args -= shift_center
        rospy.loginfo(args)
        args = list(args.ravel())
        # args.insert(0, self.move_action_name)
        self.last_angle = args[-1]  # saving last angle
        if move_type in ["move", "move_odometry"]:
            args.insert(0, move_type)
            self.add_action_node(parent_name, "move", self.move_publisher_name, self.move_response, *args)

        # TEMP cmd_publisher

    def add_command_action(self, parent_name, *args):
        self.add_action_node(parent_name, "cmd", "cmd_publisher", self.cmd_response, *args)

    def add_remove_heap_request(self, parent_name, heap_num):
        self.add_action_node(parent_name, "remove_heap", "map_publisher", self.cmd_response, "rm", heap_num)

    def add_big_action(self, parent_name, action_name, place):
        main_seq_name = self.construct_string(action_name, self.get_next_id())
        self.bt.add_node_by_string(self.construct_string(parent_name, "sequence", main_seq_name, sep=' '))
        self.add_move_action(main_seq_name, *place)

    def add_switch_secondary(self, parent_name):
        main_seq_name = self.construct_string("switch", self.get_next_id())
        self.add_sequence_node(parent_name, main_seq_name)

        if self.side == "orange":
            self.add_action_node(main_seq_name, "move", self.move_publisher_name, self.move_response, "move", 1.207, 0.25,
                                 0.79)
            self.add_command_action(main_seq_name, 162, 0, 0, 0.75, 0, 0, 6)
            self.add_command_action(main_seq_name, 182, 1) # manipulator
            self.add_command_action(main_seq_name, 224, 0) # collision avoidance
            self.add_command_action(main_seq_name, 162, -0.16, 0, 0, 0.2, 0, 0)
            self.add_command_action(main_seq_name, 162, 0.2, 0, 0, 0.57, 0, 0)
            self.add_command_action(main_seq_name, 224, 1) # collision avoidance
            self.add_command_action(main_seq_name, 182, 0) # manipulator
        else: # TODO: change coords
            self.add_action_node(main_seq_name, "move", self.move_publisher_name, self.move_response, "move", 1.947, 0.25,
                                 0.79)
            self.add_command_action(main_seq_name, 162, 0, 0, 0.75, 0, 0, 6)
            self.add_command_action(main_seq_name, 182, 1) # manipulator
            self.add_command_action(main_seq_name, 224, 0) # collision avoidance
            self.add_command_action(main_seq_name, 162, -0.16, 0, 0, 0.2, 0, 0)
            self.add_command_action(main_seq_name, 162, 0.2, 0, 0, 0.57, 0, 0)
            self.add_command_action(main_seq_name, 224, 1) # collision avoidance
            self.add_command_action(main_seq_name, 182, 0) # manipulator

    def add_switch_main(self, parent_name):
        main_seq_name = self.construct_string("switch", self.get_next_id())
        self.add_sequence_node(parent_name, main_seq_name)

        if self.side == "orange":
            self.add_action_node(main_seq_name, "move", self.move_publisher_name, self.move_response, "move", 1.22, 0.35, 1.57)
            self.add_command_action(main_seq_name, 182, 2) # manipulator
            self.add_command_action(main_seq_name, 224, 0) # collision avoidance
            self.add_command_action(main_seq_name, 162, -0.2, 0, 0, 0.2, 0, 0)
            self.add_command_action(main_seq_name, 162, 0.2, 0, 0, 0.57, 0, 0)
            self.add_command_action(main_seq_name, 182, 0)  # manipulator
            self.add_command_action(main_seq_name, 224, 0) # collision avoidance
        else: # TODO: change coords
            self.add_action_node(main_seq_name, "move", self.move_publisher_name, self.move_response, "move", 1.96, 0.35, 1.57)
            self.add_command_action(main_seq_name, 182, 2) # manipulator
            self.add_command_action(main_seq_name, 224, 0) # collision avoidance
            self.add_command_action(main_seq_name, 162, -0.2, 0, 0, 0.2, 0, 0)
            self.add_command_action(main_seq_name, 162, 0.2, 0, 0, 0.57, 0, 0)
            self.add_command_action(main_seq_name, 182, 0)  # manipulator
            self.add_command_action(main_seq_name, 224, 0) # collision avoidance

    def add_bee_secondary(self, parent_name):
        main_seq_name = self.construct_string("bee", self.get_next_id())
        self.add_sequence_node(parent_name, main_seq_name)

        if self.side == "orange":
            self.add_action_node(main_seq_name, "move", self.move_publisher_name, self.move_response, "move", 0.25, 1.75, 0)
            self.add_command_action(main_seq_name, 224, 0) # collision avoidance
            self.add_command_action(main_seq_name, 182, 2) # manipulator
            self.add_command_action(main_seq_name, 162, -0.1, 0.3, 0, 0.07, 0.2, 0)
            self.add_command_action(main_seq_name, 162, 0.1, 0.025, 0, 0.57, 0.15, 0)
            self.add_command_action(main_seq_name, 162, 0, 0, -0.8, 0, 0, 6)
            self.add_command_action(main_seq_name, 162, 0.2, -0.2, 0, 0.57, 0.57, 0)
            self.add_command_action(main_seq_name, 182, 0) # manipulator
            self.add_command_action(main_seq_name, 224, 1) # collision avoidance

        else:
            self.add_action_node(main_seq_name, "move", self.move_publisher_name, self.move_response, "move", 2.75,
                                 1.75, -0.6)
            self.add_command_action(main_seq_name, 224, 0)  # collision avoidance
            self.add_command_action(main_seq_name, 182, 2)  # manipulator
            self.add_command_action(main_seq_name, 162, 0, 0.25, 0, 0, 0.2, 0)
            self.add_command_action(main_seq_name, 162, -0.1, -0.075, 0, 0.57, 0.45, 0)
            self.add_command_action(main_seq_name, 162, 0, -0.25, 0, 0, 0.57, 0)
            self.add_command_action(main_seq_name, 182, 0)  # manipulator
            self.add_command_action(main_seq_name, 162, 0, 0, 2, 0, 0, 6)
            self.add_command_action(main_seq_name, 224, 1)  # collision avoidance

    def add_bee_main(self, parent_name):
        main_seq_name = self.construct_string("bee", self.get_next_id())
        self.add_sequence_node(parent_name, main_seq_name)

        if self.side == "orange":
            self.add_action_node(main_seq_name, "move", self.move_publisher_name, self.move_response, "move", 0.42, 1.6, -0.8)
            self.add_command_action(main_seq_name, 224, 0) # collision avoidance
            self.add_command_action(main_seq_name, 182, 1) # manipulator
            self.add_command_action(main_seq_name, 162, -0.3, -0.05, 0, 0.3, 0.05, 0)
            self.add_command_action(main_seq_name, 162, 0, 0, -1, 0, 0, 6)
            self.add_command_action(main_seq_name, 162, -0.025, 0.2, 0, 0.08, 0.57, 0)
            self.add_command_action(main_seq_name, 162, 0.2, 0, 0, 0.57, 0, 0)
            self.add_command_action(main_seq_name, 182, 0) # manipulator
            self.add_command_action(main_seq_name, 224, 1) # collision avoidance

        else: # TODO: change coords
            self.add_action_node(main_seq_name, "move", self.move_publisher_name, self.move_response, "move", 2.65, 1.53, 4.71)
            self.add_command_action(main_seq_name, 224, 0) # collision avoidance
            # self.add_command_action(main_seq_name, 182, 1) # manipulator
            self.add_command_action(main_seq_name, 162, -0.35, 0.35, 0, 0.2, 0.2, 0)
            self.add_command_action(main_seq_name, 162, 0, -0.25, 0, 0.57, 0, 0)
            # self.add_command_action(main_seq_name, 162, -0.025, -0.2, 0, 0.08, 0.57, 0)
            # self.add_command_action(main_seq_name, 162, 0.2, 0, 0, 0.57, 0, 0)
            # self.add_command_action(main_seq_name, 182, 0) # manipulator
            self.add_command_action(main_seq_name, 162, 0.35, -0.1, 0, 0.57, 0.15, 0)
            self.add_command_action(main_seq_name, 224, 1) # collision avoidance

    def add_wire_start(self, parent_name):
        node_name = self.construct_string('wire_start', self.get_next_id())
        self.add_command_action(parent_name, node_name, self.wire_start_name)

    def add_sleep_time(self, parent_name, time):
        node_name = self.construct_string('sleep', self.get_next_id())
        node_description = self.construct_string(parent_name, 'timeout', node_name, time, sep=' ')
        rospy.loginfo(node_description)
        self.bt.add_node_by_string(node_description)

    def get_angle_to_cubes(self, cubes):
        manipulator = 0
        color = 0
        for m, c in enumerate(cubes):
            if len(c) > 0:
                manipulator = m
                color = c[0]
        manipulator = 2 - manipulator
        angle = (color - manipulator + 1) % 4 * np.pi / 2 + self.black_angle
        rospy.loginfo(self.construct_string(cubes, manipulator, color, (color - manipulator + 1) % 4, sep=' '))
        return angle

    def get_mans_and_colors(self, cubes):
        mans_colors = [(i, c) for i, c in enumerate(cubes) if len(c) > 0]
        manipulators = list(zip(*mans_colors)[0])
        colors = list(zip(*mans_colors)[1])
        return manipulators, colors

    def add_pick_action(self, parent_name, m):
        if self.man_load[m] < 4:
            self.add_command_action(parent_name, self.com["pick"]["name"], m)
        else:
            self.add_command_action(parent_name, self.com["pick_last_cube"]["name"], m)
        self.man_load[m] += 1

    def add_cubes_pick(self, parent_name, heap_num, manipulators, colors, **kvargs):
        delay = 1
        if "delay" in kvargs:
            delay = kvargs["delay"]
        new = False
        if "new" in kvargs:
            new = kvargs["new"]
        doors = True
        if "doors" in kvargs:
            doors = kvargs['doors']

        rospy.loginfo(colors)
        pick_seq_name = self.construct_string("pick_with_doors", heap_num, self.get_next_id())
        self.add_sequence_node(parent_name, pick_seq_name)
        if doors:
            parallel_door_open_name = self.construct_string("parallel_door_open", heap_num, *manipulators)
            self.bt.add_node_by_string(self.construct_string(pick_seq_name, "parallel", parallel_door_open_name, sep=' '))
            self.add_command_action(parallel_door_open_name, 178, 0, 2)
            self.add_command_action(parallel_door_open_name, 178, 2, 2)

        if len(manipulators) == 1:
            self.add_pick_action(pick_seq_name, manipulators[0])
        else:
            parallel_name = self.construct_string("parallel", heap_num, *manipulators)
            self.bt.add_node_by_string(self.construct_string(pick_seq_name, "parallel", parallel_name, sep=' '))
            if delay > 0 and 0 in manipulators and 2 in manipulators:
                for m in manipulators:
                    if m == 2:
                        seq_name = self.construct_string("delay_for_2", self.get_next_id())
                        self.add_sequence_node(parallel_name, seq_name)
                        self.add_sleep_time(seq_name, delay)
                        self.add_pick_action(seq_name, m)
                    else:
                        self.add_pick_action(parallel_name, m)
            else:
                for m in manipulators:
                    self.add_pick_action(parallel_name, m)
        if doors:
            parallel_door_close_name = self.construct_string("parallel_door_close", heap_num, *manipulators)
            self.bt.add_node_by_string(self.construct_string(pick_seq_name, "parallel", parallel_door_close_name, sep=' '))
            self.add_command_action(parallel_door_close_name, 178, 0, 0)
            self.add_command_action(parallel_door_close_name, 178, 2, 0)

        for c in colors:
            self.colors_left.remove(c[0] if not new else c)
        return

    def add_sequence_node(self, parent_name, new_name):
        ss = self.construct_string(parent_name, "sequence", new_name, sep=' ')
        rospy.loginfo(ss)
        self.bt.add_node_by_string(ss)

    def add_rf_move(self, parent_name, heap_status, colors=[], mans=[]):
        # add heap_status
        rospy.loginfo("RF MOVE colors "+str(colors))
        rospy.loginfo("RF MOVE mans "+str(mans))
        rospy.loginfo("RF MOVE colors "+str(colors+mans))
        self.add_action_node(parent_name, "rf_move", "move_publisher", self.move_response, "MOVETOHEAP", heap_status, *(colors + mans))

    def add_move_to_heap(self, parent_name, heap_num, angle):
        move_seq_name = self.construct_string("move_to_heap", heap_num, self.get_next_id())
        self.add_sequence_node(parent_name, move_seq_name)
        # heap_coords = self.action_places["heaps"][heap_num][:2].tolist() + [angle]
        # self.add_move_action(move_seq_name, *heap_coords, shift_multiplier=1)  # 3
        self.add_action_node(move_seq_name,"move_heap_by_nav","move_publisher",self.move_response,"move_heap",heap_num,angle)
        # self.add_move_action(move_seq_name, *heap_coords, move_type="move_odometry")
        # self.add_rf_move(move_seq_name, 0)
        # self.add_action_node(move_seq_name, "rf_move", "move_publisher", self.move_response, "MOVETOHEAP")

    def add_heap_rotation(self, parent_name, angle):
        radius = 0.06  # m
        linear_speed = 0.05  # m/s
        self.last_coordinates[-1] += angle
        self.last_coordinates[-1] %= 2 * np.pi
        angle = angle % (2 * np.pi)
        angle = (angle + np.pi) % (2 * np.pi) - np.pi
        angle = np.fix(angle * 1000) / 1000
        self.add_command_action(parent_name, 162, radius * angle, 0, angle, linear_speed, 0, linear_speed / radius)

    def add_new_heap_rotation(self, parent_name, da):
        if da == 0:
            return
        move_seq_name = self.construct_string("heap_rotation", self.get_next_id())
        self.add_sequence_node(parent_name, move_seq_name)
        PARAMS_PI2P = [1.019 * np.pi / 2, 0.096, 0.014]
        PARAMS_PI2M = [-1.02 * np.pi / 2, -0.096, 0.01]
        if da > 0:
            a, x, y = PARAMS_PI2P
        else:
            a, x, y = PARAMS_PI2M
        w = 3
        for i in range(np.abs(da)):
            self.add_command_action(move_seq_name, 162, x, y, a, x/a*w, y/a*w, w)

    def get_heap_status(self, angle, mans=[]):
        all_colors = {0, 1, 2, 3, 4}
        # !!
        # angle from 0 to 2 pi
        if angle < 0:
            angle = np.pi - angle
        side = np.round(angle / np.pi * 2)
        if len(self.colors_left) == 5:
            return 0
        if len(self.colors_left) == 4:
            picked_color = list(all_colors - self.colors_left)[0]
            if self.side == "orange":
                if picked_color in [1, 3]:
                    picked_color = 4 - picked_color
            dif = side - picked_color
            if dif == 0:
                return 2
            if abs(dif) == 2:
                return 0
            if dif == 1:
                return 3
            if dif == -1:
                return 1
        if len(self.colors_left) == 3:
            pc = list(all_colors - self.colors_left)
            rospy.loginfo(pc)
            rospy.loginfo(side)
            if abs(pc[0] - pc[1]) == 2:
                if side - pc[0] == 0 or side - pc[1] == 0:
                    return 2
                else:
                    return 5
            else:
                first = min(pc)
                rospy.loginfo("first %d"%(first))
                rospy.loginfo("side %d"%(side))
                if 3 in pc and first == 0:
                    first = 3
                if first == side:
                    return 3
                if side == (first + 1) % 4:
                    return 1
                if side == (first + 2) % 4:
                    return 4
                if side == (first + 3) % 4:
                    return 6
        if len(self.colors_left) == 2:
            last = list(self.colors_left - {4})[0]
            rospy.loginfo("COLOR LEFT "+str(last))
            return mans[0] + 4
            # if last == side:
            #     return 5
            # if side == (last + 1) % 4:
            #     return 4
            # if side == (last + 2) % 4:
            #     return 10
            # if side == (last + 3) % 4:
            #     return 6
        if len(self.colors_left) == 1:
            color_left = list(self.colors_left)[0]
            rospy.loginfo("COLOR_LEFT "+str(color_left))
            return mans[0] + 7

            # if color_left == 4:
            #     if len(mans) > 0:
            #         rospy.loginfo("MANS " + str(mans[0]))
            #         m = mans[0]
            #         return m + 7
            #     return 10
            # if side == color_left:
            #     return 8
            # if side == (color_left + 1) % 4:
            #     return 9
            # return 7
        return 0

    def get_heap_status_new(self, mans, colors):
        if len(mans) == 1:
            return mans[0] + 7, [colors[0]], [mans[0]]
        else:
            m_best = mans[0]
            c_best = colors[0]

            for m,c in zip(mans,colors):
                rospy.loginfo(m)
                if m in [0,2]:
                    m_best = m
                    c_best = c
            return m_best + 7, [c_best], [m_best]

    def add_new_heap_pick(self, parent_name, heap_num, heap_strat, next_heap_num):
        main_seq_name = self.construct_string(parent_name, heap_strat[-1], heap_num)
        self.add_sequence_node(parent_name, main_seq_name)
        heap_strat = heap_strat[0]

        self.colors_left = {0, 1, 2, 3, 4}
        heap = self.action_places["heaps"][heap_num]
        a, c, m = self.heap_sides[heap_num]
        rospy.loginfo("move_to_heap with params %d %d %d"%self.heap_sides[heap_num])
        self.add_command_action(main_seq_name, 224, 0)  # collision avoidance

        self.add_move_to_heap(main_seq_name, heap_num, a*np.pi/2)
        self.add_remove_heap_request(main_seq_name, heap_num)
        if heap_strat[0][2] != a:
            self.add_rf_move(main_seq_name, 0, [c], [m])
        for i, (dx, dy, da, (colors, mans)) in enumerate(heap_strat):
            rospy.loginfo("--------COLORS MANS  " + str(colors) +' '+ str(mans))
            if i == 0:
                a_old = a
                a = da
                da = (a - a_old)%4
                if da == 3:
                    da = -1
                rospy.loginfo("!!!!!!!!!!!!!!!!A ALERT %d %d" % (a, a_old))
                if da != 0:
                    self.add_new_heap_rotation(main_seq_name, da)
            if da != 0 and i != 0:
                a += da
                self.add_new_heap_rotation(main_seq_name, da)
            if dx ** 2 + dy ** 2 > 0:
                rospy.loginfo("SHIFTS " + str(self.shifts.index((dx, dy))))
                rospy.loginfo(a)
                # ndx, ndy = self.rotate((dx, dy), a).tolist()
                ndx, ndy = self.shifts[(self.shifts.index((dx, dy)) - a + 1) % 4]
                rospy.loginfo((ndx, ndy))
                dX = np.array([ndx, ndy, 0]) * 5.8
                self.add_command_action(main_seq_name, 162, ndx*0.058, ndy*0.058, 0, 0.15, 0.15, 0)
                #self.add_move_action(main_seq_name, *dX.tolist(), move_type="move_stm")
            self.add_sleep_time(main_seq_name, 0.5)
            rospy.loginfo(a)
            rospy.loginfo(mans)

            if self.pick_one_by_one:
                for m, c in zip(mans, colors):
                    self.add_rf_move(main_seq_name, *self.get_heap_status_new([m], [c]))
                    self.add_cubes_pick(main_seq_name, heap_num, [m], [c], new=True, doors=False)
            else:
                if 0 in mans and 2 in mans:
                    mans_sep = [mans[:1], mans[1:]]
                    colors_sep = [colors[:1], colors[1:]]
                    for mans,colors in zip(mans_sep, colors_sep):
                        self.add_rf_move(main_seq_name, *self.get_heap_status_new(mans, colors))
                        self.add_cubes_pick(main_seq_name, heap_num, mans, colors, new=True, doors=False)
                else:
                    self.add_rf_move(main_seq_name, *self.get_heap_status_new(mans, colors))
                    #self.add_sleep_time(main_seq_name, 5)
                    self.add_cubes_pick(main_seq_name,heap_num, [m for m in mans], colors, new=True, doors=False)
        # if heap_num != None:
        #     next_a,_,_ = self.heap_sides[heap_num]
        #     da = next_a - a
        #     if da != 0:
        #         self.add_new_heap_rotation(main_seq_name, da)

        self.add_command_action(main_seq_name, 224, 1)  # collision avoidance

    def add_full_heap_pick(self, parent_name, heap_num, cubes2):
        main_seq_name = self.construct_string("heap", heap_num, self.get_next_id())
        self.add_sequence_node(parent_name, main_seq_name)

        self.colors_left = {0, 1, 2, 3, 4}
        i4 = -1  # hold the step, when we pick central cube
        for i, cubes in enumerate(cubes2):
            for c in cubes:
                if len(c) > 0 and c[0] == 4:
                    i4 = i
            manipulators, colors = self.get_mans_and_colors(cubes)
            line_seq_name = self.construct_string("pick", heap_num, *(colors + [self.get_next_id()]))
            # rospy.loginfo(line_seq_name)
            self.bt.add_node_by_string(self.construct_string(main_seq_name, "sequence", line_seq_name, sep=' '))

            if i == 0:
                self.add_move_to_heap(line_seq_name, heap_num, self.get_angle_to_cubes(cubes))
                self.add_cubes_pick(line_seq_name, heap_num, manipulators, colors, delay=1)
            elif i4 == -1:
                self.add_heap_rotation(line_seq_name, self.get_angle_to_cubes(cubes) - self.last_coordinates[-1])
                self.add_rf_move(line_seq_name, self.get_heap_status(self.last_coordinates[-1]))
                self.add_cubes_pick(line_seq_name, heap_num, manipulators, colors, delay=1.5)

            elif i4 == i and i != len(cubes2) - 1:
                # hmm....
                next_mans, next_colors = self.get_mans_and_colors(cubes2[i + 1])

                # calculate angle for picking 4: take it from side opposite to last cube
                cubes_for_angle = copy.deepcopy(cubes)
                rospy.loginfo(next_colors)
                cubes_for_angle[manipulators[0]][0] = next_colors[0][0]
                angle_to_pick_4 = (self.get_angle_to_cubes(cubes_for_angle) + np.pi) % (2 * np.pi)
                rospy.loginfo(angle_to_pick_4)
                coordinate_to_pick_4 = self.action_places["heaps"][heap_num].copy()
                coordinate_to_pick_4[-1] = angle_to_pick_4
                # self.add_move_action(line_seq_name, *coordinate_to_pick_4.ravel())
                self.add_heap_rotation(line_seq_name, angle_to_pick_4 - self.last_coordinates[-1])

                # rotate vector [58,0], not robot!!!
                # delta_xy = rot_matrix(angle_to_pick_4 + np.pi/2).dot(self.cube_vector)
                delta_xy = rot_matrix(angle_to_pick_4 - np.pi / 2 * (manipulators[0] - 1)).dot(self.cube_vector)
                # rospy.loginfo(delta_xy.flatten().tolist())
                coordinate_to_pick_4 = coordinate_to_pick_4.reshape(3, 1)
                coordinate_to_pick_4 += delta_xy
                coordinate_to_pick_4[-1] = angle_to_pick_4
                # self.add_move_action(line_seq_name, *coordinate_to_pick_4.ravel())
                # now relative motion for stm
                self.add_move_action(line_seq_name, *coordinate_to_pick_4.ravel(), move_type="move_stm")
                self.add_rf_move(line_seq_name, self.get_heap_status(self.last_coordinates[-1]) + 20)
                self.add_cubes_pick(line_seq_name, heap_num, manipulators, colors)

            else:
                #  last_cube
                rospy.loginfo("LAST CUBE " + str(self.last_angle) + ' ' + str(manipulators))

                last_cube_delta_xy = rot_matrix(self.last_angle - np.pi / 2 * (manipulators[0] - 1)).dot(
                    self.cube_vector)
                last_cube_delta_xy = rot_matrix(self.last_angle).dot(last_cube_delta_xy)

                # maybe fix
                last_cube_delta_xy = rot_matrix(-np.pi / 2 * (manipulators[0] - 1)).dot(self.cube_vector)
                rospy.loginfo("LAST CUBE " + str(last_cube_delta_xy))

                # rospy.loginfo(self.last_coordinates)
                coordinate_to_pick_5 = np.array(self.last_coordinates).reshape(3, 1)
                coordinate_to_pick_5 += last_cube_delta_xy
                # coordinate_to_pick_5[-1] = angle_to_pick_4
                # now relative motion for stm 
                self.add_move_action(line_seq_name, *coordinate_to_pick_5.ravel(), move_type="move_stm")
                self.add_rf_move(line_seq_name, self.get_heap_status(self.last_coordinates[-1], manipulators))
                self.add_cubes_pick(line_seq_name, heap_num, manipulators, colors)

        self.add_remove_heap_request(parent_name, heap_num)

    def add_cubes_sequence_new(self, heap_strat):
        self.heaps_sequence = heap_strat

    def add_cubes_sequence(self, cubes2_full):
        self.heaps_sequence = []
        it_begin = 0
        cubes_picked = 0
        for i, cubes in enumerate(cubes2_full):
            for c in cubes:
                cubes_picked += len(c)
            if cubes_picked == 5:
                self.heaps_sequence.append(cubes2_full[it_begin:i + 1])
                it_begin = i + 1
                cubes_picked = 0

        for h in self.heaps_sequence:
            print(h)
    def add_magic_cubes_action(self, parent_name):
        magic_seq_name = self.construct_string("magic_cubes_action", self.get_next_id())
        self.add_sequence_node(parent_name, magic_seq_name)

        parallel_down = self.construct_string("parallel", "shift_down_mans", self.get_next_id())
        self.bt.add_node_by_string(self.construct_string(magic_seq_name, "parallel", parallel_down, sep=' '))

        self.add_command_action(parallel_down, 179, 0)
        self.add_command_action(parallel_down, 179, 1)
        self.add_command_action(parallel_down, 179, 2)

        parallel_magic = self.construct_string("parallel", "release_magic", self.get_next_id())
        self.bt.add_node_by_string(self.construct_string(magic_seq_name, "parallel", parallel_magic, sep=' '))

        self.add_command_action(parallel_magic, self.magic_cube_action_name, 0)
        self.add_command_action(parallel_magic, self.magic_cube_action_name, 2)



    def add_disposal_action(self, parent_name, odometry_shift=False):
        super_parallel = self.construct_string("parallel", "shift_down_mans", self.get_next_id())
        self.bt.add_node_by_string(self.construct_string(parent_name, "parallel", super_parallel, sep=' '))

        main_seq_name = self.construct_string("disposal", self.get_next_id())
        self.add_sequence_node(super_parallel, main_seq_name)

        self.add_magic_cubes_action(super_parallel)

        coordinates_first = np.array([80.0, 10.0, 3.14])
        # self.add_command_action(parent_name, 0xb2, 2, 0)
        # self.add_command_action(parent_name, 0xb2, 1, 0)
        if odometry_shift:
            self.add_command_action(main_seq_name, 162, 0, 0.3, 0, 0, 0.1, 0)

        self.add_command_action(main_seq_name, 224, 0)  # collision avoidance
        self.add_move_action(main_seq_name, *coordinates_first)

        self.add_command_action(main_seq_name, 0xb2, 0, 1)  # open left
        self.add_command_action(main_seq_name, 177, 0)
        self.add_command_action(main_seq_name, 0xb3, 0)
        self.add_command_action(main_seq_name, 162, -0.15, 0, 0, 0.5, 0, 0)
        self.add_command_action(main_seq_name, 162, 0.04, 0, 0, 0.2, 0, 0)
        self.add_command_action(main_seq_name, 162, 0, -0.2, 0, 0, 0.2, 0)
        self.add_command_action(main_seq_name, 162, 0.33, 0, 0, 0.3, 0, 0)
        self.add_command_action(main_seq_name, 162, 0, 0.22, 0, 0, 0.5, 0)
        self.add_command_action(main_seq_name, 177, 1)
        self.add_command_action(main_seq_name, 0xb3, 1)
        self.add_command_action(main_seq_name, 0xb2, 2, 1)  # open right
        self.add_command_action(main_seq_name, 177, 2)
        self.add_command_action(main_seq_name, 0xb3, 2)
        self.add_command_action(main_seq_name, 162, -0.07, 0, 0, 0.3, 0, 0)
        self.add_command_action(main_seq_name, 162, 0, -0.5, 0, 0, 0.3, 0)
        self.add_command_action(main_seq_name, 0xb2, 0, 0)  # close
        self.add_command_action(main_seq_name, 0xb2, 2, 0)  # close


        self.add_command_action(main_seq_name, 224, 1)  # collision avoidance

    def test_main_robot_movements(self, parent_name):
        main_seq_name = self.construct_string("main_test", self.get_next_id())
        self.add_sequence_node(parent_name, main_seq_name)

        heap_index = 0
        heap_side = 0
        self.add_action_node(main_seq_name, "move_heap", self.move_publisher_name, self.move_response, "move_heap",
                             heap_index, heap_side)
        #some_coords = [0,0,0]
        #self.add_move_action(main_seq_name, *some_coords, move_type="move_odometry", shift_multiplier=0)

    def add_open_or_close_all_action(self, parent_name, do=0):
        # 0 - open, 1 - close
        main_seq_name = self.construct_string("open_all_action", self.get_next_id())
        self.add_sequence_node(parent_name, main_seq_name)
        man_action = 177 if do == 0 else 176
        door_ac_type = 1 if do == 0 else 0

        parallel_name1 = self.construct_string("parallel", self.get_next_id())
        self.bt.add_node_by_string(self.construct_string(main_seq_name, "parallel", parallel_name1, sep=' '))

        for i in range(3):
            self.add_command_action(parallel_name1, man_action, i)

        parallel_name2 = self.construct_string("parallel", self.get_next_id())
        self.bt.add_node_by_string(self.construct_string(main_seq_name, "parallel", parallel_name2, sep=' '))

        for i in [0,2]:
            self.add_command_action(parallel_name2, 178, i, door_ac_type)

    def add_shooting_motor_action(self, parent_name, to="left", turn="on"):
        self.add_command_action(parent_name, self.shooting_motor, 0 if to == "left" else 1, 1 if turn == "on" else 0)

    def add_shoot_sort_action(self, parent_name, to="left", delay=0.5):
        # small robot
        main_seq_name = self.construct_string("shoot_sort", self.get_next_id())
        self.add_sequence_node(parent_name, main_seq_name)
        self.add_command_action(main_seq_name, self.bottom_sorter, self.shoot_poses[to])
        if to in ["left", "right"]:
            self.add_sleep_time(main_seq_name, delay)
            self.add_command_action(main_seq_name, self.bottom_sorter, 2)

    def add_first_sort_action(self, parent_name, to="clean", delay=0.5):
        main_seq_name = self.construct_string("first_sort", self.get_next_id())
        self.add_sequence_node(parent_name, main_seq_name)
        self.add_command_action(main_seq_name, self.upper_sorter, 0 if to == "clean" else 1)
        self.add_sleep_time(main_seq_name, delay)
        self.add_command_action(main_seq_name, self.upper_sorter, 2)

    def add_wastewater_action(self, parent_name, to="release"):
        self.add_command_action(parent_name, self.wastewater_door, 1 if to == "release" else 0)

    def add_move_to_tower_action(self, parent_name, tower_name, only_odom=False):
        main_seq_name = self.construct_string("move_to_tower", self.get_next_id())
        self.add_sequence_node(parent_name, main_seq_name)

        wt_coords = self.action_places[tower_name][0]

        if not only_odom:
            # if tower_name == "wastewater_tower":
            #     shift = self.wt_y_shift
            #     wt_coords_init = wt_coords + shift if wt_coords[1] > 150 else -shift
            # else:
            #     shift = self.wt_x_shift
            #     wt_coords_init = wt_coords - shift
            # self.add_move_action(main_seq_name, *wt_coords_init.tolist(), shift_multiplier=0)
            if self.side == "orange":
                tower_index = 2 if tower_name == "wastewater_tower" else 0
            else:
                tower_index = 1 if tower_name == "wastewater_tower" else 3
            self.add_action_node(main_seq_name, "move_tower", self.move_publisher_name, self.move_response, "move_tower",tower_index)

        if only_odom:
            wt_shift = wt_coords + (self.wt_y_shift if tower_name == "wastewater_tower" else (
                self.wt_x_shift if self.side == "orange" else -self.wt_x_shift)) / 5
            wt_shift += (self.wt_x_shift if wt_coords[1] > 150 else -self.wt_x_shift) if tower_name == "wastewater_tower" else (
                -self.wt_y_shift)

            self.add_move_action(main_seq_name, *wt_shift.tolist(), move_type="move_odometry", shift_multiplier=0)
            self.add_move_action(main_seq_name, *wt_coords.tolist(), move_type="move_odometry", shift_multiplier=0)

    def add_wastewater_tower(self, parent_name, delay=1):
        main_seq_name = self.construct_string("wastewater_tower", self.get_next_id())
        self.add_sequence_node(parent_name, main_seq_name)

        self.add_command_action(main_seq_name, self.upper_sorter, self.first_poses["interm"])
        self.add_command_action(main_seq_name, self.bottom_sorter, self.shoot_poses["interm"])
        self.add_move_to_tower_action(main_seq_name, "wastewater_tower")
        self.add_command_action(main_seq_name, 224, 0)  # collision avoidance
        self.add_command_action(main_seq_name, self.bottom_sorter, 2)
        self.add_command_action(main_seq_name, self.wastewater_door, 0)
        delay = 0.5
        for i in range(4):
            self.add_command_action(main_seq_name, self.upper_sorter, self.first_poses["interm"])
            self.add_command_action(main_seq_name, 162, -0.004, 0, 0, 0.57, 0, 0)
            self.add_sleep_time(main_seq_name, .1)
            self.add_command_action(main_seq_name, self.upper_sorter, self.first_poses["interm waste"])
            self.add_command_action(main_seq_name, 162, 0.008, 0, 0, 0.57, 0, 0)
            self.add_sleep_time(main_seq_name, .1)
            self.add_command_action(main_seq_name, self.upper_sorter, self.first_poses["interm clean"])
            self.add_command_action(main_seq_name, 162, -0.004, 0, 0, 0.57, 0, 0)
            self.add_sleep_time(main_seq_name, .1)

            # self.add_first_sort_action(main_seq_name, "clean")
            # self.add_command_action(main_seq_name, self.upper_sorter, self.first_poses["interm clean"])
            self.add_sleep_time(main_seq_name, delay)
            self.add_command_action(main_seq_name, self.upper_sorter, self.first_poses["clean"])
            self.add_sleep_time(main_seq_name, delay)
            self.add_command_action(main_seq_name, self.upper_sorter, self.first_poses["interm waste"])
            self.add_sleep_time(main_seq_name, delay)
            if i != 3:
                self.add_command_action(main_seq_name, self.upper_sorter, self.first_poses["waste"])
                self.add_sleep_time(main_seq_name, delay)

        self.add_command_action(main_seq_name, 224, 1)  # collision avoidance

    def add_wastewater_reservoir(self, parent_name):
        main_seq_name = self.construct_string("wastewater_reservoir", self.get_next_id())
        self.add_sequence_node(parent_name, main_seq_name)
        self.add_command_action(main_seq_name, 224, 0) # collision avoidance
        if self.side == "orange":
            self.add_command_action(main_seq_name, 162, 0.05, 0.1, 0, 0.57, 0.57, 0)
            self.add_command_action(main_seq_name, 162, 0, 0, 3, 0, 0, 6)
            self.add_command_action(main_seq_name, 162, -0.12, 0.05, 0, 0.16, 0.5, 0)
        else:
            self.add_command_action(main_seq_name, 162, -0.22, 0, 0.1, 0.25, 0, 0.08)

        self.add_wastewater_action(main_seq_name, "release")
        self.add_sleep_time(main_seq_name,1)
        self.add_first_sort_action(main_seq_name, "waste")
        self.add_sleep_time(main_seq_name, 2)
        self.add_command_action(main_seq_name, 224, 1) # collision avoidance

        self.add_command_action(main_seq_name, 162, 0.05, 0.05, 0, 0.57, 0.57, 0)
        if self.side == "orange":
            self.add_command_action(main_seq_name, 162, 0.05, -0.05, 0, 0.57, 0.57, 0)
        else:
            self.add_command_action(main_seq_name, 162, 0.05, 0.05, 0, 0.3, 0.57, 0)

        self.add_wastewater_action(main_seq_name, "close")

        if self.side == "orange":
            self.add_command_action(main_seq_name, 162, 0.1, -0.1, 0, 0.57, 0.57, 0)
        else:
            self.add_command_action(main_seq_name, 162, 0.1, 0.2, 0, 0.3, 0.57, 0)


    def add_cleanwater_tower(self, parent_name, to="left", with_4_balls=False, only_4_balls=False):
        main_seq_name = self.construct_string("cleanwater_tower", self.get_next_id())
        self.add_sequence_node(parent_name, main_seq_name)

        self.add_command_action(main_seq_name, self.upper_sorter, self.first_poses["interm"])
        if not with_4_balls:
            self.add_shoot_sort_action(main_seq_name, "release " + to)
        self.add_move_to_tower_action(main_seq_name, "cleanwater_tower", False) #not with_4balls
        self.add_shooting_motor_action(main_seq_name, to, "on")
        self.add_command_action(main_seq_name, 224, 0) # collision avoidance
        if with_4_balls:
            for _ in range(4):
                self.add_shoot_sort_action(main_seq_name, to, .8)
                self.add_sleep_time(main_seq_name, .8)
        if not only_4_balls:
            self.add_shoot_sort_action(main_seq_name, "release " + to)
            for _ in range(8):
                #self.add_first_sort_action(main_seq_name, "clean", .5)
                # self.add_command_action(main_seq_name, self.upper_sorter, self.first_poses["interm clean"])
                # self.add_sleep_time(main_seq_name, .1)
                self.add_command_action(main_seq_name, self.upper_sorter, self.first_poses["interm"])
                self.add_command_action(main_seq_name, 162, -0.004, 0, 0, 0.57, 0, 0)
                self.add_sleep_time(main_seq_name, .15)
                self.add_command_action(main_seq_name, self.upper_sorter, self.first_poses["interm waste"])
                self.add_command_action(main_seq_name, 162, 0.008, 0, 0, 0.57, 0, 0)
                self.add_sleep_time(main_seq_name, .15)
                self.add_command_action(main_seq_name, self.upper_sorter, self.first_poses["interm clean"])
                self.add_command_action(main_seq_name, 162, -0.004, 0, 0, 0.57, 0, 0)
                self.add_sleep_time(main_seq_name, .15)
                self.add_command_action(main_seq_name, self.upper_sorter, self.first_poses["clean"])
                self.add_sleep_time(main_seq_name, .5)

                # self.add_shoot_sort_action(main_seq_name, to, .8)

        self.add_sleep_time(main_seq_name, .5)

        self.add_shooting_motor_action(main_seq_name, to, "off")
        self.add_command_action(main_seq_name, 224, 1) # collision avoidance
        self.add_command_action(main_seq_name, 162, 0, 0.1, 0, 0, 0.57, 0)

    def time_checker(self, parent_name, time):
        time_node_name = self.construct_string("time_checker", self.get_next_id())

        def time_checker_fun():
            if self.bt.nodes[parent_name].time_worked() > time:
                return 2
            else:
                return 0
        self.bt.add_node(ActionFunctionNode(time_node_name, time_checker_fun), parent_name)

    def add_strategy(self, strategy):
        self.strategy_sequence = strategy

    def create_tree_from_strategy(self, wire_start=False):
        ss = self.construct_string(self.bt.name, "sequence", self.root_seq_name, sep=' ')
        rospy.loginfo(ss)
        self.bt.add_node_by_string(ss)

        if wire_start:
            self.add_wire_start(self.root_seq_name)

        for name, num in self.strategy_sequence:
            if name == 'base':
                continue
            elif name == "start_move":
                self.add_command_action(self.root_seq_name, 162, +0.2, 0, 0, 0.2, 0, 0)
            elif name == "help":
                self.add_open_or_close_all_action(self.root_seq_name, num)
            elif name == 'disposal':
                self.add_disposal_action(self.root_seq_name, False)
            elif name in ['switch_secondary']:
                self.add_switch_secondary(self.root_seq_name)
            elif name in ['bee_secondary']:
                self.add_bee_secondary(self.root_seq_name)
            elif name in ['switch_main']:
                self.add_switch_main(self.root_seq_name)
            elif name in ['bee_main']:
                self.add_bee_main(self.root_seq_name)
            elif name == 'heaps':
                next_num = num[1]
                num = num[0]
                self.add_new_heap_pick(self.root_seq_name, num, self.heaps_sequence[num], next_num)
            elif name == 'cleanwater_tower_after_waste':
                self.add_cleanwater_tower(self.root_seq_name, "left" if self.side == "orange" else "right", True, False)
            elif name == 'cleanwater_tower_before_waste':
                self.add_cleanwater_tower(self.root_seq_name, "left" if self.side == "orange" else "right", False,
                                          False)
            elif name == 'cleanwater_tower_only_shoot4':
                self.add_cleanwater_tower(self.root_seq_name, "left" if self.side == "orange" else "right", False, True)
            elif name == "wastewater_tower":
                self.add_wastewater_tower(self.root_seq_name)
            elif name == "wastewater_reservoir":
                self.add_wastewater_reservoir(self.root_seq_name)
            elif name == "test_main":
                self.test_main_robot_movements(self.root_seq_name)
            elif name == "time":
                self.time_checker(self.root_seq_name, num)
            elif name == "magic_cubes":
                self.add_magic_cubes_action(self.root_seq_name)
        return self.bt


import sys
from strategy_operator import StrategyOperator

if __name__ == "__main__":
    rospy.init_node("btb_node", anonymous=True)
    move_pub = rospy.Publisher("/main_robot/move_command", String, queue_size=100)
    cmd_pub = rospy.Publisher("/main_robot/stm_command", String, queue_size=100)
    map_pub = rospy.Publisher("/map_server/cmd", String, queue_size=10)
    move_type = 'standard'
    response_sub = SubscriberHandler("/main_robot/response")
    btb = BehaviorTreeBuilder("main_robot", move_pub, cmd_pub, map_pub, response_sub, response_sub,
                              move_type=move_type)
    # btb.add_strategy([("heaps",1),("funny",1),("heaps",2),("heaps",0),("disposal",0),("funny",0)])
    # btb.add_strategy([("heaps", 0), ("heaps", 1), ("heaps", 2), ("disposal", 0)])
    # btb.add_strategy([("disposal",0)])
    # btb.add_strategy([("bee_main",0), ("heaps", 1), ("heaps", 0), ("heaps", 2)])
    # btb.add_strategy([("bee_main",0), ("switch_main",0), ("heaps", (1,0)), ("heaps", (0,2)), ("heaps", (2,None))])
    # btb.add_strategy([("heaps", 0)])
    # btb.add_strategy([("heaps", 0),("heaps", 1),("heaps", 2)])
    btb.add_strategy([("switch_main", 0)])
    # btb.add_strategy([("heaps", 0),("heaps", 1)])
    # btb.add_strategy([("heaps", (0,1)),("heaps", (1,None)),  ("disposal", 0)])#, ("switch_main", 0), ("bee_main", 0)])
    # so = StrategyOperator(file='first_bank.txt')

    # btb.add_cubes_sequence(so.get_cubes_strategy(['orange','black','green'])[0])
    rospy.loginfo("AAAAAAAAAAAAAAa")
    rospack = rospkg.RosPack()
    with open(rospack.get_path('eurobot_decision_maker') + "/scripts/cubes_paths_beta_1.bin","rb") as f:
        heap_strats = pickle.load(f)
    # btb.add_cubes_sequence([[[], [0], []],
    #                         [[3], [2], [1]],
    #                         [[4], [], []],
    #                         [[2], [], []],
    #                         [[1], [0], [3]],
    #                         [[], [], [4]],
    #                         [[], [], [3]],
    #                         [[2], [1], [0]],
    #                         [[], [4], []]])
    # # [[], [], [4]],
    # [[], [], [3]]])
    rospy.loginfo(heap_strats[0]['012'])
    btb.add_cubes_sequence_new(heap_strats[0]['012'])
    btb.pick_one_by_one = True
    btb.create_tree_from_strategy(wire_start=False)
    #print(heap_strats[1]['001'])
    rospy.sleep(1)

    btb.bt.root_node.start()
    # btb.man_load[0] = 3
    # btb.man_load[1] = 4
    # btb.man_load[2] = 3
    r = rospy.Rate(10)
    while not rospy.is_shutdown() and btb.bt.root_node.check_status() != "finished":
        r.sleep()
