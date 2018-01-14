#! /usr/bin/env python

import rospy
from std_msgs.msg import String

import re

pub = None


class TickEngine:
	def __init__(self, initial_state):
		self.state = initial_state
	
	def tick(self):
		pass
	
	def wait(self):
		pass


class TreeNode:
	def __init__(self, parent):
		self.status = False 	# True if active right now
		self.parent = parent
	
		
class ControlNode(TreeNode):
	def __init__(self, parent):
		


if __name__  == '__main__':
    rospy.init_node('fake_stm', anonymous =True)
    
    pub = rospy.Publisher('bt_maker', String, queue_size = 10)
    #sub = rospy.Subscriber('robot_command', String, fake_response)
    data = ""
    with open("tree_description.txt") as btf:
        data = btf.read().replace('\n',' ')
    rospy.loginfo(data)
    pub.publish(data)
    rospy.spin()
