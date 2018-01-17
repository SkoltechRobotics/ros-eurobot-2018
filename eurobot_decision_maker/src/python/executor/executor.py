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
        rospy.loginfo( self.name + ' started!' )
        self.time_start = rospy.get_time()

    def finish(self):
        self.time_finish = rospy.get_time()
        rospy.loginfo( self.name + ' finished! time ' + str(self.time_finish - self.time_start))

    def check_status(self):
        return self.status

    def time_worked(self):
        if self.time_finish != 0:
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
    def __init__(self,command_id, command_publisher, message, request_topic_name):
        
        # super(ActionNode, self).__init__()
        # doesn't work ??!! -> replaced with
        TreeNode.__init__(self, command_id)

        self.id = command_id
        # unique command id
        # it's also $self.name

        self.command_pub = command_publisher     
        # this is ros publisher

        self.request_topic_name = request_topic_name    
        # this is name, subscriber will be created later
        
        self.message = message
        # this is python string

    def callback_for_terminating(self):
        def cb(msg):
            action_id, action_status = re.match("(\S*)\s(\S*)",msg.data).group(1,2) # finish it!
            if action_id == self.id:
                self.status = action_status
                if self.status == "finished":
                    self.finish()
        return cb

    def start(self):
        if self.status == "not started": # do we need some mutex here?
            TreeNode.start(self)
            self.status = "active"
            self.command_pub.publish(self.id + ' ' + self.message)
            self.sub = rospy.Subscriber(self.request_topic_name, String, self.callback_for_terminating())

    def finish(self):
        TreeNode.finish(self)
        self.sub.unregister()
	
    def tick(self):
        if self.status == "not started":
            self.start()
        
        return self.status
		
class ControlNode(TreeNode):
    def __init__(self, name):
    	TreeNode.__init__(self, name)
        self.children_list = []
        
    def append_child(self, child):
        self.children_list.append(child)

    def tick(self):
        if self.status == "not started":
            self.start()
            self.status = "active"

    
class SequenceNode(ControlNode):
    def __init__(self, name):
        ControlNode.__init__(self, name)
        
    def tick(self):
        
        ControlNode.tick(self) # set status="active"

        child_iter = iter(self.children_list)
        child = None
        try:
            child = child_iter.next()
        except StopIteration:
            # empty list!
            print "Empty children list in " + self.name + " !"
            self.finish()
            self.status = "error"
            raise
        except:
            print "Unexpected error in " + self.name
            raise
        
        while child.check_status() == "finished":
            try:
                child = child_iter.next()
            except StopIteration:
                # all children finished
                self.finish()
                self.status = "finished"
                return self.status
            except:
                print "Unexpected error in " + self.name
                raise
        
        current_child_status = child.check_status()
        if current_child_status in ["active", "not started"]:
            child.tick()
            return self.status

class ParallelNode(ControlNode):
    def __init__(self, name, maximum_failed = 0):
        ControlNode.__init__(self, name)
        self.maximum_failed = maximum_failed

    def tick(self):
        ControlNode.tick(self) # set status="active"
        
        children_status = [child.tick() for child in self.children_list]
        
        if not "active" in children_status and not "running" in children_status:
            N_failed = sum([1 if ch_status == "error" else 0 for ch_status in children_status])
            if N_failed <= self.maximum_failed:
                self.status = "finished"
            else:
                self.status = "error"

        return self.status

class TimeoutNode(TreeNode):
    def __init__(self, name, sleep_time):
        TreeNode.__init__(self, name)
        self.sleep_time = sleep_time

    def tick(self):
        if self.status == "not started":
            self.start()
            self.status = "active"

        
        if self.status == "active" and self.time_worked() >= sleep_time:
            self.finish()


if __name__  == '__main__':
    rospy.init_node('executor', anonymous=True)
    
    pub = rospy.Publisher("fake_stm_command", String, queue_size=100)
    a1 = ActionNode("a1", pub, "move 0 0 0", "fake_stm_response")
    a2 = ActionNode("a2", pub, "move 0 0 1", "fake_stm_response")
    a3 = ActionNode("a3", pub, "move 0 0 2", "fake_stm_response")

    s0 = SequenceNode("s0")
    # s1 = SequenceNode("s1")
    p0 = ParallelNode("p0")
    s0.append_child(a1)
    s0.append_child(p0)
    p0.append_child(a2)
    p0.append_child(a3)
    rospy.sleep(0.2)
    
    
    r = rospy.Rate(100)
    while s0.status != "finished":
        s0.tick()
        r.sleep()
        
    
    rospy.loginfo("a1 time " + str(a1.time_worked()))
    rospy.loginfo("a2 time " + str(a2.time_worked()))
    rospy.loginfo("s0 time " + str(s0.time_worked())) 
    rospy.loginfo("a3 time " + str(a3.time_worked()))
    rospy.loginfo("p0 time " + str(p0.time_worked()))
    

