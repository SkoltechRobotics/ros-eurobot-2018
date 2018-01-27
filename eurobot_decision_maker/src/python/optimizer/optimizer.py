#! /usr/bin/env python
# coding: utf-8

# In[42]:


import gurobipy as grb
import numpy as np
import itertools
import sys
#sys.path.append("~/catkin_ws/src/ros-eurobot-2018/eurobot_decision_maker/src/python/executor")
from executor import *


# In[11]:


def iterer(*args):
    return itertools.product(*[range(x_) for x_ in args])


# In[35]:


class StrategyOptimizer:
    """
        Single (big) robot optimizer
    """
    actions = {
        "heaps": 6,
        "funny": 2,
        "disposal": 1,
        "base": 1
    }

    action_places = {
        "heaps": np.array([[85, 54, 0], [30, 119, 0], [110, 150, 0], [190, 150, 0], [270, 119, 0], [215, 54, 0]]),
        "funny": np.array([[113, 10, 0], [10, 190, 0]]),
        "disposal": np.array([[61, 10, 0]]),
        "base": np.array([[15, 15, 0]])
    }
    def __init__(self, **kvargs):
        self.model = grb.Model("eurobot")
        self.actions = {}
        
        # choose some certain actions
        # by default all from Optimizer.actions
        if "actions" in kvargs:
            try:
                for a in kvargs["actions"]:
                    self.actions[a] = StrategyOptimizer.actions[a]
            except:
                print("Error: wrong actions in kvargs")
        else:
            self.actions.update(StrategyOptimizer.actions)
        
        
        # N for total available actions (from them we can choose)
        self.N = sum(self.actions.values())
        # S for total maximal number of actions can be created by optimizer
        self.S = self.N - 3
        self.x = self.model.addVars(range(self.S), range(self.N), name = 'x', vtype=grb.GRB.BINARY)
        
        #define positions of all actions in x variable
        self.actions_iters = {}
        i = 0
        for a in self.actions:
            self.actions_iters[a] = list(range(i,i+self.actions[a]))
            i += self.actions[a]
        print(self.actions_iters)
        self.extra_conditions = []
       
        # set default Time Matrix
        self.createDefaultTimeMatrix()
        
        # create time constraint/objective
        self.createTimeConstraint()
        
        # create points contraint/objective
        self.createPointsConstraint()
        
        self.Time = 100.0
        self.Points = 220.0
    
        self.objective = "time"
        if "objective" in kvargs:
            if kvargs["objective"] in ["time","points"]:
                self.objective = kvargs["objective"]
        
    def resetConstraints(self):
        self.model.remove(self.model.getConstrs())
        
    def createDefaultTimeMatrix(self):
        ## Calculate time matrix
        # places with cubes
        #heap_points = np.array([[54, 85, 0], [119, 30, 0], [150, 110, 0], [150, 190, 0], [119, 270, 0], [54, 215, 0]])

        # starting place of robot
        # to be redefined at the start of the algorithm
        #base_point  = np.array([[15, 15, 0]])

        # disposal place 
        # TODO: add another side of field
        #disposal_point = np.array([[10, 61, 0]])

        # funny action places
        #funny_points = np.array([[10, 113, 0], [190, 10, 0]])

        action_places = np.concatenate(tuple(StrategyOptimizer.action_places[action] for action in self.actions ))

        # time is simple distance / v
        v = 20

        self.time_travel = np.sum((action_places[np.newaxis,:]-action_places[:,np.newaxis]) ** 2, axis = 2)**0.5 / v
        
    def addDefaultConstraints(self):
        # unique action per 's'-tep
        self.model.addConstrs((self.x.sum(s, '*') == 1 for s in iterer(self.S)));
        
        # each action can be done only once
        self.model.addConstrs((self.x.sum('*', i) <= 1 for i in range(self.N)));
            
        # now max 3 heaps
        self.model.addConstr(grb.quicksum((self.x.sum('*',i) for i in iterer(self.actions["heaps"]))) <= 3 );
        
        # disposal after all 3 heaps picking
        for s, k in iterer(self.S,self.actions["heaps"]):
            self.model.addConstr( grb.quicksum( (self.x[s_,self.actions_iters["disposal"][0]] for s_ in range(s,self.S)) )
                                 >= self.x[s,k] );
        # starting from base point
        self.model.addConstr(self.x[0,self.actions_iters["base"][0]] == 1);
        
    def createTimeConstraint(self):
        time_travel_constr_gen = (self.x[s, k_1] * self.x[s+1, k_2] * self.time_travel[k_1, k_2] for  s,k_1, k_2 in 
                                  #[(0,9, 2)])
                                  iterer(self.S-1,self.N,self.N))
        left_part_time_1 = grb.quicksum(time_travel_constr_gen)

        self.left_part_time = left_part_time_1
    
    def createPointsConstraint(self):
        #TODO: rewrite with points array
        self.total_points = 45*grb.quicksum((self.x.sum('*',k) for k in self.actions_iters["heaps"])) +                                          30*self.x.sum('*',self.actions_iters["funny"][0]) +                                          55*self.x.sum('*',self.actions_iters["funny"][1]);
    def setObjective(self):
        """
            minimize "time"
            or
            maximize "points"
        """
        if self.objective=="time":
            self.model.addConstr(self.total_points == self.Points, 'points')
            self.model.setObjective(self.left_part_time, grb.GRB.MINIMIZE)
        elif self.objective=="points":
            self.model.addConstr(self.left_part_time <= self.Time, 'time')
            self.model.setObjective(self.total_points, grb.GRB.MAXIMIZE)
    
    def run(self):
        self.resetConstraints()
        self.addDefaultConstraints()
        self.setObjective()
        
        self.model.update()
        self.model.optimize()
        self.model.printAttr('x')

    def getSolution(self, **kvargs):
        view = "numbers"
        if "view" in kvargs and kvargs['view'] in ["numbers", "text"]:
            view = kvargs["view"]
        
        result = []
        
        for a in self.x.items():
            if abs(a[1].X - 1) < 0.1:
                result.append(a[0][1])

        if view == "numbers":
            return result
        else:
            result_tuples = []
            for a in result:
                for name, indices in self.actions_iters.items():
                    if a in indices:
                        result_tuples.append((name,a-indices[0]))
            return result_tuples

# In[36]:

if __name__ == "__main__":
    opt = StrategyOptimizer()
    opt.run()
    print(opt.getSolution(view="text"))

