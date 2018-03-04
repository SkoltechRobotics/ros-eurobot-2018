#! /usr/bin/env python

from executor import *
# from optimizer import *
import copy
import numpy as np
#from cube_picking_optimizer import *

def rot_matrix(rot_angle):
    return np.matrix([  [np.cos(rot_angle), -np.sin(rot_angle), 0],
                        [np.sin(rot_angle),  np.cos(rot_angle), 0],
                        [0,                  0,                 1]])


class BehaviorTreeBuilder:
    scale = {"m":1000.0,"dm":100.0,"cm":10.0,"mm":1.0 }
    def __init__(self, bt_name, move_pub, cmd_pub, move_response, cmd_response, **kvargs):
        self.id = 0
        self.bt = BehaviorTree(bt_name)
        self.root_seq_name = 'strategy'
        self.bt.add_publisher("move_publisher", move_pub)
        self.bt.add_publisher("cmd_publisher", cmd_pub)
        self.move_response = move_response
        self.cmd_response = cmd_response
        self.black_angle = 0 #angle for black cube to be picked by central manipulator
        self.pick_action_name = str(176)
        self.last_coordinates = []
        self.last_angle = self.black_angle
        self.heaps_sequences = []
        self.strategy_sequence = []
        self.cube_vector = np.array([[0],[5.8],[0.0]])
        self.opt_units = "cm" # from self.opt_units to self.track_units
        self.track_units = "m"
        self.move_action_name = str(0x0E)
        self.move_publisher_name = "cmd_publisher"
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
        self.action_places = {
                "heaps": np.array([[54, 85, 0], [119, 30, 0], [150, 110, 0], [150, 190, 0], [119, 270, 0], [54, 215, 0]], dtype=np.float64),
                "funny": np.array([[10, 113, 0], [190, 10, 0]], dtype=np.float64),
                "disposal": np.array([[10, 61, 0]],dtype=np.float64),
                "base": np.array([[15, 15, 0]],dtype=np.float64)
            }

        for _, action in self.action_places.items():
            for coords in action:
                coords[:2] = coords[:2][::-1]
        self.colors_left = {0,1,2,3,4}

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
    def add_action_node(self, parent_name,prefix, str_pub,str_response, *args):
        # node_name = prefix+str(self.get_next_id())
        node_name = self.construct_string(prefix, self.get_next_id())
        # node_description = parent_name + ' ' + 'action' + ' ' + node_name + str_pub + ' ' + str_response 
        # for arg in args:
        #     node_description += ' ' + str(arg)
        node_description = self.construct_string(parent_name, 'action', node_name, str_pub, str_response, *args, sep=' ')
        rospy.loginfo(node_description)
        self.bt.add_node_by_string(node_description)
    
    def convert_units(self, dist):
        return BehaviorTreeBuilder.scale[self.opt_units]/BehaviorTreeBuilder.scale[self.track_units]*dist
    def add_move_action(self, parent_name, *args, **kvargs):
        move_type = "move"
        if "move_type" in kvargs and kvargs["move_type"] in ["move", "move_odometry"]:
            move_type = kvargs["move_type"]
        shift_multiplier = 1
        if "shift_multiplier" in kvargs:
            shift_multiplier = kvargs["shift_multiplier"]
        args = list(args) 
        robot_angle = args[-1]
        self.last_coordinates = copy.deepcopy(args)
        rospy.loginfo(args)
        args[:2] = [self.convert_units(a) for a in args[:2]]
        args = np.array(args).reshape(3,1)
        shift_center = rot_matrix(robot_angle).dot(self.convert_units(self.cube_vector*shift_multiplier))
        rospy.loginfo(shift_center)
        args -= shift_center
        rospy.loginfo(args)
        args = list(args.ravel())
        # args.insert(0, self.move_action_name)
        args.insert(0, move_type)
        self.last_angle = args[-1] # saving last angle
        # TEMP cmd_publisher
        self.add_action_node(parent_name, "move", self.move_publisher_name, self.move_response, *args)
    def add_command_action(self, parent_name, *args):
        self.add_action_node(parent_name, "cmd", "cmd_publisher", self.cmd_response, *args)
    def add_big_action(self, parent_name, action_name, place):
        main_seq_name = self.construct_string(action_name, self.get_next_id())
        self.bt.add_node_by_string(self.construct_string(parent_name, "sequence", main_seq_name, sep=' ')) 
        self.add_move_action(main_seq_name, *place)
    

    def get_angle_to_cubes(self, cubes):
        manipulator = 0
        color = 0
        for m,c in enumerate(cubes):
            if len(c) > 0:
                manipulator = m
                color = c[0]

        angle =  (color-manipulator+1) % 4 * np.pi/2 + self.black_angle
        rospy.loginfo(self.construct_string(cubes,manipulator,color,(color-manipulator+1) % 4 ,sep=' '))
        return 2*np.pi - angle
    def get_mans_and_colors(self, cubes):
        mans_colors = [(i,c) for i,c in enumerate(cubes) if len(c) > 0]
        manipulators = list(zip(*mans_colors)[0])
        colors = list(zip(*mans_colors)[1])
        return manipulators, colors


    def add_cubes_pick(self, parent_name, heap_num, manipulators, colors):
        if len(manipulators) == 1:
            self.add_command_action(parent_name, self.pick_action_name, manipulators[0])
        else:
            parallel_name = self.construct_string("parallel", heap_num, *manipulators)
            self.bt.add_node_by_string(self.construct_string(parent_name, "parallel", parallel_name, sep=' '))
            for m in manipulators:
                self.add_command_action(parallel_name, self.pick_action_name, m)
        for c in colors:
            self.colors_left.remove(c[0])
        return
    def add_sequence_node(self, parent_name, new_name):
        ss = self.construct_string(parent_name, "sequence", new_name, sep=' ')
        rospy.loginfo(ss)
        self.bt.add_node_by_string(ss)
        
    def add_rf_move(self, parent_name, heap_status):
        # add heap_status
        self.add_action_node(parent_name, "rf_move", "move_publisher", self.move_response, "MOVETOHEAP", heap_status)

    def add_move_to_heap(self, parent_name, heap_num, angle):
        move_seq_name = self.construct_string("move_to_heap", heap_num, self.get_next_id())
        self.add_sequence_node(parent_name, move_seq_name)
        heap_coords = self.action_places["heaps"][heap_num][:2].tolist() + [angle]
        self.add_move_action(move_seq_name, *heap_coords, shift_multiplier=3)
        self.add_move_action(move_seq_name, *heap_coords, move_type="move_odometry")
        self.add_rf_move(move_seq_name, 0)
        # self.add_action_node(move_seq_name, "rf_move", "move_publisher", self.move_response, "MOVETOHEAP")

    def add_heap_rotation(self, parent_name, angle):
        radius = 0.06       #m
        linear_speed = 0.05 #m/s
        self.last_coordinates[-1] += angle
        self.last_coordinates[-1] %= 2*np.pi
        angle = angle % (2*np.pi)
        angle = np.fix(angle*1000)/1000
        self.add_command_action(parent_name, 162, radius*angle, 0, angle,  linear_speed, 0, linear_speed/radius)

    def get_heap_status(self, angle):
        all_colors = {0,1,2,3,4}
        if angle < 0:
            angle = 2*np.pi - angle
        side = np.round(angle/np.pi*2)
        if len(self.colors_left) == 5:
            return 0
        if len(self.colors_left) == 4:
            picked_color = list(all_colors - self.colors_left)[0]
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
            if abs(pc[0]-pc[1]) == 2:
                if side - pc[0] == 0 or side - pc[1] == 0:
                    return 5
                else:
                    return 2
            else:
                first = min(pc)
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
            if last == side:
                pass
        if len(self.colors_left) == 1:
            color_left = list(self.colors_left)[0]
            if color_left == 4:
                return 10

        return 0

    def add_full_heap_pick(self, parent_name, heap_num, cubes2):
        main_seq_name = self.construct_string("heap", heap_num, self.get_next_id())
        self.add_sequence_node(parent_name, main_seq_name) 

        self.colors_left = {0,1,2,3,4}
        i4 = -1 # hold the step, when we pick central cube
        for i,cubes in enumerate(cubes2):
            for c in cubes:
                if len(c) > 0 and c[0] == 4:
                    i4 = i
            manipulators, colors = self.get_mans_and_colors(cubes)
            line_seq_name = self.construct_string("pick", heap_num, *(colors + [self.get_next_id()]))
            # rospy.loginfo(line_seq_name)
            self.bt.add_node_by_string(self.construct_string(main_seq_name, "sequence", line_seq_name,sep=' '))    
            
            if i == 0:
                # first move from outside to the heap and pick
                rospy.loginfo("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
                self.add_move_to_heap(line_seq_name, heap_num, self.get_angle_to_cubes(cubes))
                self.add_cubes_pick(line_seq_name, heap_num, manipulators, colors)
            elif i4 == -1:
                # simply rotate and pick
                # rospy.loginfo(*(self.action_places["heaps"][heap_num][:2].tolist() + [1] ))
                # self.add_move_action(line_seq_name, *(self.action_places["heaps"][heap_num][:2].tolist() + [ self.get_angle_to_cubes(cubes) ]))
                self.add_heap_rotation(line_seq_name, self.get_angle_to_cubes(cubes) - self.last_coordinates[-1])
                self.add_rf_move(line_seq_name, self.get_heap_status(self.last_coordinates[-1]))
                self.add_cubes_pick(line_seq_name, heap_num, manipulators, colors)
            
            elif i4 == i and i != len(cubes2) - 1:
                # hmm....
                next_mans, next_colors = self.get_mans_and_colors(cubes2[i+1])
                
                # calculate angle for picking 4: take it from side opposite to last cube
                cubes_for_angle = copy.deepcopy(cubes)
                rospy.loginfo(next_colors)
                cubes_for_angle[manipulators[0]][0] = next_colors[0][0]
                angle_to_pick_4 = (self.get_angle_to_cubes(cubes_for_angle) + np.pi) % (2*np.pi)
                rospy.loginfo(angle_to_pick_4)
                coordinate_to_pick_4 = self.action_places["heaps"][heap_num].copy()
                coordinate_to_pick_4[-1] = angle_to_pick_4
                # self.add_move_action(line_seq_name, *coordinate_to_pick_4.ravel())
                self.add_heap_rotation(line_seq_name, angle_to_pick_4 - self.last_coordinates[-1])

                # rotate vector [58,0], not robot!!!
                # delta_xy = rot_matrix(angle_to_pick_4 + np.pi/2).dot(self.cube_vector)
                delta_xy = rot_matrix(angle_to_pick_4 - np.pi/2*(manipulators[0]-1)).dot(self.cube_vector)
                #rospy.loginfo(delta_xy.flatten().tolist())
                coordinate_to_pick_4 = coordinate_to_pick_4.reshape(3,1)
                coordinate_to_pick_4 += delta_xy
                coordinate_to_pick_4[-1] = angle_to_pick_4
                # self.add_move_action(line_seq_name, *coordinate_to_pick_4.ravel())
                # now relative motion for stm
                self.add_move_action(line_seq_name, *coordinate_to_pick_4.ravel(), move_type="move_odometry")

                self.add_cubes_pick(line_seq_name, heap_num, manipulators, colors)
            
            else:
                #  last_cube
                last_cube_delta_xy = rot_matrix(self.last_angle - np.pi/2*(manipulators[0] - 1)).dot(self.cube_vector)
                #rospy.loginfo(self.last_coordinates)
                coordinate_to_pick_5 = np.array(self.last_coordinates).reshape(3,1)
                coordinate_to_pick_5 += last_cube_delta_xy
                # coordinate_to_pick_5[-1] = angle_to_pick_4
                # now relative motion for stm 
                self.add_move_action(line_seq_name, *coordinate_to_pick_5.ravel(), move_type="move_odometry")
                self.add_cubes_pick(line_seq_name, heap_num, manipulators, colors)
                   
    def add_cubes_sequence(self, cubes2_full):
        self.heaps_sequence = []
        it_begin = 0
        cubes_picked = 0
        for i,cubes in enumerate(cubes2_full):
            for c in cubes:
                cubes_picked += len(c)
            if cubes_picked == 5:
                self.heaps_sequence.append(cubes2_full[it_begin:i+1])
                it_begin = i+1
                cubes_picked = 0

        for h in self.heaps_sequence:
            print(h)
        
            

    def add_strategy(self, strategy):
        self.strategy_sequence = strategy

    def create_tree_from_strategy(self):
        ss = self.construct_string(self.bt.name, "sequence", self.root_seq_name,sep=' ')
        rospy.loginfo(ss)
        self.bt.add_node_by_string(ss)

        for name, num in self.strategy_sequence:
            if name == 'base':
                continue
            elif name in ['funny','disposal']:
                self.add_big_action(self.root_seq_name, self.construct_string(name,num), self.action_places[name][num])
            elif name == 'heaps':
                self.add_full_heap_pick(self.root_seq_name, num, self.heaps_sequence[num])

        return self.bt


        



                
import sys
from strategy_operator import StrategyOperator

if __name__ == "__main__":
    rospy.init_node("btb_node", anonymous=True)
    move_pub = rospy.Publisher("/main_robot/move_command", String, queue_size=100)
    cmd_pub  = rospy.Publisher("/main_robot/stm_command",  String, queue_size=100)
    move_type = 'standard'
    if len(sys.argv) > 1 and sys.argv[1] in ['simple','standard']:
        move_type = sys.argv[1]
    btb = BehaviorTreeBuilder("main_robot", move_pub, cmd_pub, "/main_robot/response", "/main_robot/response", move_type=move_type)
    # btb.add_strategy([("heaps",1),("funny",1),("heaps",2),("heaps",0),("disposal",0),("funny",0)])
    btb.add_strategy([("heaps",0),("heaps",1),("heaps",2)])

    # btb.add_strategy([("heaps",0)])
    
    #so = StrategyOperator(file='first_bank.txt')
    
    # btb.add_cubes_sequence(so.get_cubes_strategy(['orange','black','green'])[0])
    
    btb.add_cubes_sequence([[[], [0], []],
                            [[1], [2], [3]],
                            [[4], [], []],
                            [[2], [], []],
                            [[3], [0], [1]],
                            [[], [], [4]],
                            [[], [], [3]],
                            [[0], [1], [2]],
                            [[], [4], []]])
                            #[[], [], [4]],
                            #[[], [], [3]]])
    
    btb.create_tree_from_strategy()
    rospy.sleep(1)
    btb.bt.root_node.start()
    r = rospy.Rate(10)
    while not rospy.is_shutdown() and btb.bt.root_node.check_status() != "finished":
        r.sleep()
