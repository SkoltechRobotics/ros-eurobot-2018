#! /usr/bin/env python

import rospy
from std_msgs.msg import String

import re
import json

def construct_string(*args, **kvargs):
        sep = "_"
        if "sep" in kvargs:
            sep = kvargs["sep"]
        node_name = ""
        for a in args[:-1]:
            node_name += str(a) + sep
        return node_name + str(args[-1])


class StrategyOperator:
    possible_plans = [
        ['orange','black','green'],
        ['yellow','black','blue'],
        ['blue','green','orange'],
        ['yellow','green','black'],
        ['black','yellow','orange'],
        ['green','yellow','blue'],
        ['blue','orange','black'],
        ['green','orange','yellow'],
        ['black','blue','green'],
        ['orange','blue','yellow']
    ]
    mapping_heaps = {
        'orange': ['black','green','blue','orange','yellow'],
        'green': ['black','orange','blue','green','yellow']
    }
    
    def __init__(self, **kvargs):
        self.optimizer = None # object with real-time optimizer
        self.bank = None # object with loaded from json-loaded saved strategies
        
        if 'file' in kvargs:
            self.read_file(kvargs['file'])
        if 'optimizer' in kvargs:
            self.optimizer = kvargs['optimizer']
            
        self.init_bank()
        self.time_params = { # to be loaded with rospy.params
            'rotate': 2,
            'pick': 2.5,
            'move': 1.5
        }
        if 'time_params' in kvargs:
            self.time_params = kvargs['time_params']
        
    def init_bank(self):
        if self.bank == None:
            self.bank = {}
            self.bank['cubes'] = {}
            self.bank['routes'] = {}
        
    def read_file(self, filename, mode='r'):
        with open(filename, mode) as f:
            self.bank = json.load(f)
    
    def save_file(self, filename, mode='w'):
        with open(filename, mode) as f:
            json.dump(self.bank, f)
            
    # colors in format ['orange','blue','green'] also 'black', 'yellow'
    # for orange and green sides there is a different cubes order
    
    def add_cubes_picking_strategy(self, strategy, time, colors, mask="ooo"):
        cubes_strategy_name = construct_string(*(colors+[mask]),sep=' ')
        self.bank['cubes'][cubes_strategy_name] = (strategy, time)
    
    def get_color_number(self,color,side='orange'):
        if isinstance(color,str):
            return self.mapping_heaps[side].index(color)
        elif isinstance(color,int) or isinstance(color,float):
            color = int(color)
            return self.mapping_heaps[side][color]
    
    def set_bank_from_cube_optimizer(self, cube_opt, with_mask=False):
        if with_mask:
            pass # TODO: improve cubes optimizer
        else:
            for color_plan in self.possible_plans:
                num_plan = [self.get_color_number(color) for color in color_plan]
                cube_opt.set_plan(num_plan)
                opt_time_params = [self.time_params[k] for k in ['rotate','move','pick']]
                timer = cube_opt.get_fun_time(*opt_time_params)
                seq = cube_opt.find_optimal_sequence(timer)
                self.add_cubes_picking_strategy(seq, timer(seq), color_plan)
    def get_cubes_strategy(self, colors, mask="ooo"):
        cubes_strategy_name = construct_string(*(colors+[mask]),sep=' ')
        if not cubes_strategy_name in self.bank['cubes']:
            return [None, None]
        else:
            return self.bank['cubes'][cubes_strategy_name]
