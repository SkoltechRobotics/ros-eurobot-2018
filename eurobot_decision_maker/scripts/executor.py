#! /usr/bin/env python

import rospy
from std_msgs.msg import String

import re


class SubscriberHandler(object):
    def __init__(self, topic_name):
        self.nodes = {}
        self.topic_name = topic_name
        rospy.Subscriber(topic_name, String, self.response_callback)

    def handle_response(self, node, name):
        self.nodes[name] = node

    def response_callback(self, data):
        action_id, action_status = re.match("(\S*)\s(\S*)", data.data).group(1, 2)  # finish it!
        if action_id in self.nodes:
            self.nodes[action_id].status = action_status
            if action_status in ["finished", "failed"]:
                self.nodes[action_id].finish()
                self.nodes.pop(action_id)
        else:
            rospy.logwarn("Node %s name not found" % action_id)


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

    def reset(self):
        self.status = "not started"

    def start(self):
        rospy.loginfo(self.name + ' started!')
        self.time_start = rospy.get_time()

    def finish(self):
        self.time_finish = rospy.get_time()
        rospy.loginfo(self.name + ' finished! time ' + str(self.time_finish - self.time_start) + ' with status ' +
                      self.status)

    def check_status(self):
        return self.status

    def time_worked(self):
        if self.time_finish != 0:
            return self.time_finish - self.time_start
        else:
            return rospy.get_time() - self.time_start


class ActionNode(TreeNode):
    """
        This class stands for simple executable Eurobot action.
        By .start() it sends specific message of format:
            $command_id + ' ' + $message
        into topic $command_topic, then 
        recieves any status changes from $request_topic_name
        
        Any status changes will be stopped after .finish() execution

        TODO: rewrite this docs in approtiate way
    """

    def __init__(self, command_id, command_publisher, message, sub_handler, without_response=False):

        # super(ActionNode, self).__init__()
        # doesn't work ??!! -> replaced with
        TreeNode.__init__(self, command_id)

        self.id = command_id
        # unique command id
        # it's also $self.name

        self.command_pub = command_publisher
        # this is ros publisher

        self.sub_handler = sub_handler
        # this is name, subscriber will be created later

        self.message = message
        # this is python string
        self.without_response = without_response

    def start(self):
        if self.status == "not started":  # do we need some mutex here?
            TreeNode.start(self)
            self.status = "active"
            if isinstance(self.message, str):
                self.message = self.id + ' ' + self.message
            else:
                self.message.goal_id.id = self.id
            rospy.loginfo(self.message)

            if not self.without_response:
                self.sub_handler.handle_response(self, self.id)
                self.command_pub.publish(self.message)
            else:
                self.command_pub.publish(self.message)
                self.status = 'finished'
                TreeNode.finish(self)

    def finish(self):
        TreeNode.finish(self)

    def tick(self):
        if self.status == "not started":
            self.start()

        return self.status


class ControlMultiChildrenNode(TreeNode):
    def __init__(self, name):
        TreeNode.__init__(self, name)
        self.children_list = []

    def append_child(self, child):
        self.children_list.append(child)

    def tick(self):
        if self.status == "not started":
            self.start()
            self.status = "active"


class ControlSingleChildNode(TreeNode):
    def __init__(self, name):
        TreeNode.__init__(self, name)
        self.child = None

    def set_child(self, child):
        self.child = child

    def tick(self):
        if self.status == "not started":
            self.start()
            self.status = "active"


class SequenceNode(ControlMultiChildrenNode):
    def __init__(self, name):
        ControlMultiChildrenNode.__init__(self, name)

    def reset(self):
        self.status = "not started"
        for child in self.children_list:
            child.reset()

    def tick(self):
        ControlMultiChildrenNode.tick(self)

        child_iter = iter(self.children_list)
        child = None
        try:
            child = child_iter.next()
        except StopIteration:
            # empty list!
            print ("Empty children list in " + self.name + " !")
            self.finish()
            self.status = "error"
            raise
        except:
            print ("Unexpected error in " + self.name)
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
                print ("Unexpected error in " + self.name)
                raise

        current_child_status = child.check_status()
        if current_child_status in ["active", "not started"]:
            child.tick()
            return self.status

        if current_child_status in ["failed", "error"]:
            self.status = current_child_status
            return current_child_status


class TryUntilSuccessNode(ControlSingleChildNode):
    def __init__(self, name, max_reset_attempts=None):
        ControlSingleChildNode.__init__(self, name)
        self.max_reset_attempts = max_reset_attempts
        self.reset_attempts = 0

    def tick(self):
        ControlSingleChildNode.tick(self)

        child_status = self.child.tick()
        if child_status in ["failed", "error"] and (self.max_reset_attempts is None or self.reset_attempts < self.max_reset_attempts):
            self.child.reset()
            self.reset_attempts += 1

        elif child_status == "finished":
            self.status = "finished"


class ActionFunctionNode(TreeNode):
    # function return status
    # 0 - finished
    # 1 - active
    # 2 - error
    def __init__(self, name, function=None):
        TreeNode.__init__(self, name)
        self.function = function

    def tick(self):
        if self.status == "not started":
            self.start()
        if self.function:
            status = self.function()
            self.status = ["finished", "active", "error"][status]
            if self.status in ["finished", "error"]:
                self.finish()


class ParallelNode(ControlMultiChildrenNode):
    def __init__(self, name, maximum_failed=0):
        ControlMultiChildrenNode.__init__(self, name)
        self.maximum_failed = maximum_failed

    def tick(self):
        ControlMultiChildrenNode.tick(self)  # set status="active"

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

        if self.status == "active" and self.time_worked() >= self.sleep_time:
            self.finish()
            self.status = "finished"

        return self.status


class SelectorNode(ControlMultiChildrenNode):
    def __init__(self, name):
        ControlMultiChildrenNode.__init__(self, name)

    def tick(self):
        ControlMultiChildrenNode.tick(self)

        child_iter = iter(self.children_list)
        child = None

        try:
            child = child_iter.next()
        except StopIteration:
            # empty list!
            print ("Empty children list in " + self.name + " !")
            self.finish()
            self.status = "error"
            raise
        except:
            print ("Unexpected error in " + self.name)
            raise

        # find first not executed child
        # previous are all failed

        while child.check_status() == "failed":
            try:
                child = child_iter.next()
            except StopIteration:
                # all children finished
                self.finish()
                self.status = "failed"
                return self.status
            except:
                print ("Unexpected error in " + self.name)
                raise

        current_child_status = child.check_status()

        # continue execution
        if current_child_status in ["active", "not started"]:
            child.tick()

        # finish execution
        if current_child_status in ["error", "finished"]:
            self.status = current_child_status

        return self.status


class RootNode(TreeNode):
    def __init__(self, name, execution_rate=100):
        # execution rate is 100Hz by default

        TreeNode.__init__(self, name)

        self.child = None
        self.execution_rate = execution_rate
        # self.looped = loop_execution
        self.timer = None

    def append_child(self, child):
        self.set_child(child)

    def set_child(self, child):
        self.child = child

    def tick(self):
        self.child.tick()

        child_status = self.child.check_status()

        if child_status in ["failed", "finished", "error"]:
            self.status = child_status
            self.finish()

    def start(self):
        if self.status == "active":
            return

        TreeNode.start(self)

        def _callback_tick(e):
            return self.tick()

        self.timer = rospy.Timer(rospy.Duration(1.0 / self.execution_rate), _callback_tick)
        self.status = "active"

    def finish(self):
        TreeNode.finish(self)

        if self.timer != None:
            self.timer.shutdown()


class BehaviorTree:
    def __init__(self, name, execution_rate=100):
        self.name = name
        self.root_node = RootNode(self.name, execution_rate)
        self.nodes = {self.name: self.root_node}
        self.node_types = {
            "action": ActionNode,
            "sequence": SequenceNode,
            "parallel": ParallelNode,
            "selector": SelectorNode,
            "wait": TimeoutNode,
            "timeout": TimeoutNode
        }

        self.pubs = {}
        self.subs = {}

    def add_publisher(self, pub_name, pub):
        self.pubs[pub_name] = pub

    def add_subscriber(self, sub_name, sub):
        self.subs[sub_name] = sub

    def add_node(self, node, parent_name):

        parent_node = None

        try:
            parent_node = self.nodes[parent_name]
        except KeyError:
            print ("Error: no such parent node in BT")
            raise
        except:
            raise

        try:
            parent_node.append_child(node)
        except:  # type of exception ?
            print ("Error: parent_node is not a ControlNode")
            raise
        else:
            self.nodes[node.name] = node

    def add_node_by_string(self, node_string):

        # first substring       parent name
        # second substring      node type
        # third substring       node name
        # rest                  node parameters

        # TODO: add constructors from parameters string!

        try:
            parent_name, node_type, node_name, node_params = re.match("(\S*)\s(\S*)\s(\S*)\s([\S\s]*)",
                                                                      node_string).group(1, 2, 3, 4)
        except AttributeError:
            try:
                parent_name, node_type, node_name = re.match("(\S*)\s(\S*)\s(\S*)", node_string).group(1, 2, 3)
            except:
                print ("Error: wrong string description")
                raise

        try:
            NodeClass = self.node_types[node_type]
        except KeyError:
            print ("Error: no such node type!")
            raise
        except:
            raise

        node = None

        if node_type == "action":
            # print node_params
            publisher_name, request_topic_name, message = re.match("(\S*)\s(\S*)\s([\S\s]*)", node_params).group(1, 2,
                                                                                                                 3)
            node = ActionNode(node_name, self.pubs[publisher_name], message, self.subs[request_topic_name])

        # elif node_type == "parallel":
        # add parameter about n_failed
        elif node_type in ["wait", "timeout"]:
            try:
                sleep_time = float(node_params)
            except:
                print ("Error: wrong string description")
            else:
                node = TimeoutNode(node_name, sleep_time)
        else:
            node = NodeClass(node_name)

        self.add_node(node, parent_name)


""" 
    pub = rospy.Publisher("fake_stm_command", String, queue_size=100)
    a1 = ActionNode("a1", pub, "move 0 0 0", "fake_stm_response")
    a2 = ActionNode("a2", pub, "move 0 0 1", "fake_stm_response")
    a3 = ActionNode("a3", pub, "move 0 0 2", "fake_stm_response")

    s0 = SequenceNode("s0")
    s1 = SequenceNode("s1")
    p0 = ParallelNode("p0")
    # s0.append_child(a1)
    # s0.append_child(p0)
    # p0.append_child(a2)
    # p0.append_child(a3)
    rospy.sleep(0.2)
    
    
    bt = BehaviorTree("big_robot", 100)

    bt.add_node(s0, "big_robot")
    bt.add_node(a1, s0.name)
    bt.add_node(p0, s0.name)
    bt.add_node(a2, p0.name)
    bt.add_node(a3, p0.name)


    
    bt.root_node.start()
    r = rospy.Rate(10)
    while bt.root_node.check_status() != "finished":
        r.sleep()

   
    bt2 = BehaviorTree("small_robot", 100)
    
    bt2.add_publisher("fake_stm_command", pub)
    bt2.add_node_by_string("small_robot sequence s0")
    bt2.add_node_by_string("s0 action a1 fake_stm_command fake_stm_response move 0 0 0")
    bt2.add_node_by_string("s0 parallel p0")
    bt2.add_node_by_string("p0 action a2 fake_stm_command fake_stm_response move 0 0 1")
    bt2.add_node_by_string("p0 action a3 fake_stm_command fake_stm_response move 0 0 2")

    bt2.root_node.start()

    while bt2.root_node.check_status() != "finished":
        r.sleep()


    rospy.loginfo("a1 time " + str(a1.time_worked()))
    rospy.loginfo("a2 time " + str(a2.time_worked()))
    rospy.loginfo("s0 time " + str(s0.time_worked())) 
    rospy.loginfo("a3 time " + str(a3.time_worked()))
    rospy.loginfo("p0 time " + str(p0.time_worked()))
    
"""
