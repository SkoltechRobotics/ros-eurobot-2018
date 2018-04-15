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
    rospy.init_node("btb_secondary_node", anonymous=True)
    move_pub = rospy.Publisher("/secondary_robot/move_command", String, queue_size=100)
    cmd_pub  = rospy.Publisher("/secondary_robot/stm_command",  String, queue_size=100)
    map_pub  = rospy.Publisher("/map_server/cmd", String, queue_size=10)
    move_type = 'standard'
    if len(sys.argv) > 1 and sys.argv[1] in ['simple','standard']:
        move_type = sys.argv[1]
    response_sub = SubscriberHandler("/secondary_robot/response")
    btb = BehaviorTreeBuilder("secondary_robot", move_pub, cmd_pub, map_pub, response_sub, response_sub, move_type=move_type)
    # btb.add_strategy([("heaps",1),("funny",1),("heaps",2),("heaps",0),("disposal",0),("funny",0)])
    btb.add_strategy([("bee_secondary",0)])
    # btb.add_strategy([("cleanwater_tower_before_waste", 0), ("switch_secondary", 0), ("wastewater_tower", 0), ("wastewater_reservoir", 0)])
    # btb.add_strategy([("cleanwater_tower_before_waste",0)])
    # btb.add_strategy([("cleanwater_tower_before_waste",0),("switch_secondary",0),("bee_secondary",0)])
    # btb.add_strategy([("cleanwater_tower_before_waste",0), ("switch_secondary",0), ("wastewater_tower", 0), ("wastewater_reservoir", 0), ("bee_secondary",0)])
    # btb.add_strategy([("cleanwater_tower_before_waste",0), ("switch_secondary",0), ("wastewater_tower",0), ("wastewater_reservoir",0), ("bee_secondary", 0)])
    # btb.add_strategy([("bee_secondary", 0)])

    #so = StrategyOperator(file='first_bank.txt')
    
    # btb.add_cubes_sequence(so.get_cubes_strategy(['orange','black','green'])[0])
    
    # btb.add_cubes_sequence([[[], [0], []],
    #                         [[3], [2], [1]],
    #                         [[4], [], []],
    #                         [[2], [], []],
    #                         [[1], [0], [3]],
    #                         [[], [], [4]],
    #                         [[], [], [3]],
    #                         [[2], [1], [0]],
    #                         [[], [4], []]])
                            #[[], [], [4]],
                            #[[], [], [3]]])
    
    btb.create_tree_from_strategy(wire_start=False)
    rospy.sleep(1)
    btb.bt.root_node.start()
    r = rospy.Rate(10)
    while not rospy.is_shutdown() and btb.bt.root_node.check_status() != "finished":
        r.sleep()
