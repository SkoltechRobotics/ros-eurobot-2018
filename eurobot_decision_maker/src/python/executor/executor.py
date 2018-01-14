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
		self.status = "not started"
		self.parent = parent
	

class ActionNode(TreeNode):
	def __init__(self,command_id, command_topic, message, request_topic):
		self.id = command_id
		self.command_pub = command_topic 	# this is ros publisher
		self.request_topic_name = request_topic # this is name, subscriber will be created later
		self.status = "not started"
		self.message = message
	def callback_for_terminating(self):
		def cb(msg):
			action_id, action_status = re.match("(\S*)\w(\S*)",msg.data).group(??) # finish it!
			if action_id == self.command_id:
				self.status = action_status
		return cb
	def start():
		self.status = "active"
		self.command_pub.publish(self.command_id + ' ' + self.message)
		self.sub = rospy.Subscriber(self.request_topic_name, String, self.callback_for_terminating())
	
	def finish():
		self.status = "finished"
		self.sub.unregister()
		
		
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
