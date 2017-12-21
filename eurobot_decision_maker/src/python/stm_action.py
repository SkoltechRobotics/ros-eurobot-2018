#! /usr/bin/env python


import rospy

import actionlib

import eurobot_decision_maker.msg 

import sys

import re


class BTAction(object):
    # create messages that are used to publish feedback/result
    _feedback = eurobot_decision_maker.msg.BTFeedback()
    _result   = eurobot_decision_maker.msg.BTResult()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, eurobot_decision_maker.msg.BTAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
                
    def execute_cb(self, goal):
        # helper variables
        print("EXECUTED CALLBACK")
        r = rospy.Rate(1)
        success = True
        # publish info to the console for the user
        rospy.loginfo('Starting Action %s' % self._action_name)
        
        # start executing the action
        for i in xrange(1, 5):
            #self.set_status('RUNNING')
          # check that preempt has not been requested by the client
            #if self._as.is_preempt_requested():
            #    rospy.loginfo('Action %s: Halted' % self._action_name)
            #    self._as.set_preempted()
            #    success = False
            #    break

            rospy.loginfo('Executing Action: %s' % self._action_name)

          # this step is not necessary, the sequence is computed at 1 Hz for demonstration purposes
            r.sleep()
          
        if success:
            self.set_status('SUCCESS')

    

    def set_status(self,status):
        if status == 'SUCCESS':
            self._feedback.status = 1
            self._result.status = self._feedback.status
            rospy.loginfo('Action %s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)
        elif status == 'FAILURE':
            self._feedback.status = 2
            self._result.status = self._feedback.status
            rospy.loginfo('Action %s: Failed' % self._action_name)
            self._as.set_succeeded(self._result)
        else:
            rospy.logerr('Action %s: has a wrong return status' % self._action_name)


#name of topic to move smth "stm_command"
#send messages in format "%action_name %type args.."

#name of topic with response "stm_response"
#recieve response in format "%action_name done"

from std_msgs.msg import String

stm_command = None
stm_response = None

class STMAction(BTAction):
    _feedback = eurobot_decision_maker.msg.BTFeedback()
    _result   = eurobot_decision_maker.msg.BTResult()


    def __init__(self, name, message , stm_pub):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, eurobot_decision_maker.msg.BTAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
        self.stm_command = stm_pub
	rospy.loginfo("EE")
        self.stm_response = rospy.Subscriber('stm_response',String, self.on_response)
        self.moving_finished = False
        self.message = message #previously sys.argv

    def on_response(self,msg):
        waiting_for = self._action_name + ' ' + 'done'
        rospy.loginfo(msg.data)
        if msg.data == waiting_for:
            self.set_status('SUCCESS')
            self.moving_finished = True

        
    def execute_cb(self, goal):
        rospy.loginfo( "EXECUTING MOVING CALLBACK" )

        r = rospy.Rate(10) #checking for 'done' response at this rate
        
        #success = True
        
        rospy.loginfo('Started Action %s' % self._action_name)
        
        rospy.loginfo(" CREATED PUB & SUB")
        
        #stm_command_str  = ""
        #for i, comm in enumerate(self.message_argv[1:]):
        #    stm_command_str += comm
        #    if i < len(self.message_argv[1:])-1:
        #        stm_command_str += ' '

        self.stm_command.publish(self.message)
        rospy.loginfo(self.message)

        # while not self.moving_finished:
            
        rospy.spin()
        
#args: 
#0 : path
#1 : name of action node
#2 : type (e.g. "move", "pick")

#other arguments specific to type of action

#if type == "move"
#3 : x coordinate
#4 : y coordinate
#5 : angle coordinate
#6 : type of moving



stm_command = None
stm_actions = []
def create_action_cb(msg):
    global stm_command
    global stm_actions
    rospy.loginfo(msg.data)
    action_name = re.match("(\S*)\s([\s\S]*)",msg.data).group(1)
    stm_actions.append(STMAction(action_name, msg.data, stm_command))
    

if __name__ == '__main__':
      
    rospy.init_node("mover")
    
    #global stm_command
    stm_command  = rospy.Publisher('robot_command',String,queue_size = 10)
    
    stm_commander = rospy.Subscriber('commander', String, create_action_cb)

    #STMAction("move1", "move1 move 0 0 0",  stm_command)
    #STMAction("move2", "move2 move 0 0 0",  stm_command)
    #STMAction("move3", "move3 move 0 0 0",  stm_command)
    
    #create_action_cb("move1 move 0 0 0")
    #create_action_cb("move2 move 0 0 0")
    #create_action_cb("move3 move 0 0 0")

    rospy.spin()
