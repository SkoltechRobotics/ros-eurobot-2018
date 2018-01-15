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
    """
        Base class for all possible nodes in tree.

        .time_worked() calculates time since .start() until now or until .finish() execution

        TODO: rewrite this in approtiate way
    """
    def __init__(self, name):
        self.status = "not started"
        self.time_start = 0
        self.time_finish = 0
        self.name = name

    def start(self):
        self.time_start = rospy.get_time()

    def finish(self):
        self.time_finish = rospy.get_time()

    def check_status(self):
        return self.status

    def time_worked(self):
        if not self.time_finish is 0:
            return self.time_finish - self.time_start
        else:
            return rospy.get_time() - self.time_start

class ActionNode(TreeNode):
    """
        This class stands for simple executable action.
        By .start() it sends specific message of format:
            $command_id + ' ' + $message
        into topic $command_topic, then 
        recieves any status changes from $request_topic_name
        
        Any status changes will be stopped after .finish() execution

        TODO: rewrite this docs in approtiate way
    """
    def __init__(self,command_id, command_topic, message, request_topic_name):
        
        # super(ActionNode, self).__init__()
        # doesn't work ??!! -> replaced with
        TreeNode.__init__(self, command_id)

        self.id = command_id
        # unique command id
        # it's also $self.name

        self.command_pub = command_topic 	        
        # this is ros publisher

        self.request_topic_name = request_topic_name    
        # this is name, subscriber will be created later
        
        self.message = message
        # this is python string

    def callback_for_terminating(self):
        def cb(msg):
            action_id, action_status = re.match("(\S*)\s(\S*)",msg.data).group(1,2) # finish it!
            if action_id == self.command_id:
                self.status = action_status
        return cb

    def start():
        if not self.status is "not started": # do we need some mutex here?
            TreeNode.start(self)
            self.status = "active"
            self.command_pub.publish(self.command_id + ' ' + self.message)
            self.sub = rospy.Subscriber(self.request_topic_name, String, self.callback_for_terminating())

    def finish():
        TreeNode.finish(self)
        self.status = "finished"
        self.sub.unregister()
	
    def tick():
        if self.status is "not started":
            self.start()
        return self.status
		
class ControlNode(TreeNode):
    def __init__(self, name):
    	TreeNode.__init__(self, name)
        child_list = []
        
    def append_child(self, child):
        child_list.append(child)

    
class SequenceNode(ControlNode):
    def __init__(self, name):
        ControlNode.__init__(self, name)
        
    def tick(self):
        if self.status is "not started":
            self.start()
        
        child_iter = iter(child_list)
        
        


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
