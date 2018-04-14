#! /usr/bin/env python

from executor import *
# from optimizer import *
import copy
import numpy as np
from bt_builder import BehaviorTreeBuilder
# from cube_picking_optimizer import *



import sys
import pickle
from strategy_operator import StrategyOperator

if __name__ == "__main__":
    rospy.init_node("btb_node", anonymous=True)
    move_pub = rospy.Publisher("/main_robot/move_command", String, queue_size=100)
    cmd_pub = rospy.Publisher("/main_robot/stm_command", String, queue_size=100)
    map_pub = rospy.Publisher("/map_server/cmd", String, queue_size=10)
    move_type = 'standard'
    res_sub = SubscriberHandler("/main_robot/response")
    btb = BehaviorTreeBuilder("main_robot", move_pub, cmd_pub, map_pub, res_sub, res_sub,
                              move_type=move_type)
    # btb.add_strategy([("heaps",1),("funny",1),("heaps",2),("heaps",0),("disposal",0),("funny",0)])
    # btb.add_strategy([("heaps", 0), ("heaps", 1), ("heaps", 2), ("disposal", 0)])
    # btb.add_strategy([("disposal",0)])
    # btb.add_strategy([("heaps",0)])
    action_name = "help"
    action_num = "open"
    if len(sys.argv) > 1:
        action_name = sys.argv[1]
    if len(sys.argv) > 2:
        action_num = int(sys.argv[2])
    btb.add_strategy([(action_name, action_num)])
    # so = StrategyOperator(file='first_bank.txt')

    # btb.add_cubes_sequence(so.get_cubes_strategy(['orange','black','green'])[0])
    with open("cubes_paths_beta_2.bin","rb") as f:
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
    btb.add_cubes_sequence_new(heap_strats[1]['012'])
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
