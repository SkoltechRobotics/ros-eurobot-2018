#! /usr/bin/env python

from executor import *
from optimizer import *
import copy
#from cube_picking_optimizer import *


class BehaviorTreeBuilder:
    def __init__(self, bt_name, move_pub, cmd_pub, move_response, cmd_response):
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
        self.cube_vector = np.array([[58],[0],[0]])
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

    def add_move_action(self, parent_name, *args):
        args = list(args) 
        self.last_coordinates = copy.deepcopy(args)
        #rospy.loginfo(args)
        args.insert(0, "MOVE")
        self.last_angle = args[-1] # saving last angle
        self.add_action_node(parent_name, "move", "move_publisher", self.move_response, *args)
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
        return (color-manipulator+1) % 4 * np.pi/2 + self.black_angle
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
        return
    
    def add_full_heap_pick(self, parent_name, heap_num, cubes2):
        main_seq_name = self.construct_string("heap", heap_num, self.get_next_id())
        ss = self.construct_string(parent_name, "sequence", main_seq_name, sep=' ')
        rospy.loginfo(ss)
        self.bt.add_node_by_string(ss)

        i4 = -1 # hold the step, when we pick central cube
        for i,cubes in enumerate(cubes2):
            for c in cubes:
                if len(c) > 0 and c[0] == 4:
                    i4 = i
            manipulators, colors = self.get_mans_and_colors(cubes)
            line_seq_name = self.construct_string("pick", heap_num, *(colors + [self.get_next_id()]))
            #rospy.loginfo(line_seq_name)
            self.bt.add_node_by_string(self.construct_string(main_seq_name, "sequence", line_seq_name,sep=' '))    

            if i4 == -1:
                # simply rotate and pick
                #rospy.loginfo(*(StrategyOptimizer.action_places["heaps"][heap_num][:2].tolist() + [1] ))
                self.add_move_action(line_seq_name, *(StrategyOptimizer.action_places["heaps"][heap_num][:2].tolist() + [ self.get_angle_to_cubes(cubes)]))
                self.add_cubes_pick(line_seq_name, heap_num, manipulators, colors)
            
            elif i4 == i and i != len(cubes2) - 1:
                # hmm....
                next_mans, next_colors = self.get_mans_and_colors(cubes2[i+1])
                
                # calculate angle for picking 4: take it from side opposite to last cube
                cubes_for_angle = copy.deepcopy(cubes)
                cubes_for_angle[manipulators[0]][0] = next_mans[0]
                angle_to_pick_4 = (self.get_angle_to_cubes(cubes_for_angle) + np.pi) % (2*np.pi)

                coordinate_to_pick_4 = StrategyOptimizer.action_places["heaps"][heap_num].copy()
                coordinate_to_pick_4[-1] = angle_to_pick_4
                self.add_move_action(line_seq_name, *coordinate_to_pick_4.ravel())
                
                # rotate vector [58,0], not robot!!!
                def rot_matrix(rot_angle):
                    return np.matrix([  [np.cos(rot_angle), -np.sin(rot_angle), 0],
                                        [np.sin(rot_angle),  np.cos(rot_angle), 0],
                                        [0,                  0,                 1]])

                delta_xy = rot_matrix(angle_to_pick_4 + np.pi/2).dot(self.cube_vector)
                #rospy.loginfo(delta_xy.flatten().tolist())
                coordinate_to_pick_4 = coordinate_to_pick_4.reshape(3,1)
                coordinate_to_pick_4 += delta_xy
                coordinate_to_pick_4[-1] = angle_to_pick_4
                self.add_move_action(line_seq_name, *coordinate_to_pick_4.ravel())
                self.add_cubes_pick(line_seq_name, heap_num, manipulators, colors)
            
            else:
                #  last_cube
                last_cube_delta_xy = rot_matrix(self.last_angle + np.pi/2*(manipulators[0] - 1)).dot(self.cube_vector)
                #rospy.loginfo(self.last_coordinates)
                coordinate_to_pick_5 = np.array(self.last_coordinates).reshape(3,1)
                coordinate_to_pick_5 += last_cube_delta_xy
                coordinate_to_pick_5[-1] = angle_to_pick_4
                self.add_move_action(line_seq_name, *coordinate_to_pick_5.ravel())
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
                self.add_big_action(self.root_seq_name, self.construct_string(name,num), StrategyOptimizer.action_places[name][num])
            elif name == 'heaps':
                self.add_full_heap_pick(self.root_seq_name, num, self.heaps_sequence[num])

        return self.bt


        



                

if __name__ == "__main__":
    rospy.init_node("btb_node", anonymous=True)
    move_pub = rospy.Publisher("/main_robot/move_command", String, queue_size=100)
    cmd_pub  = rospy.Publisher("/main_robot/cmd_command",  String, queue_size=100)
    btb = BehaviorTreeBuilder("main_robot", move_pub, cmd_pub, "/main_robot/response", "/main_robot/cmd_response")
    btb.add_strategy([("heaps",1),("funny",0),("heaps",2),("heaps",0),("disposal",0),("funny",0)])
    btb.add_cubes_sequence([[[0], [1], []],
                            [[3], [], []],
                            [[4], [], []],
                            [[2], [], []],
                            [[], [3], [0]],
                            [[], [], [1]],
                            [[], [4], []],
                            [[], [2], []],
                            [[0], [1], [2]],
                            [[], [], [4]],
                            [[], [], [3]]])
    btb.create_tree_from_strategy()
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        r.sleep()
