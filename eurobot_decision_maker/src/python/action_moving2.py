#! /usr/bin/env python


import rospy

import actionlib

import behavior_tree_core.msg




class BTAction(object):
    # create messages that are used to publish feedback/result
    _feedback = behavior_tree_core.msg.BTFeedback()
    _result   = behavior_tree_core.msg.BTResult()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, behavior_tree_core.msg.BTAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
        
    def execute_cb(self, goal):
        # helper variables
        print("EXECUTED action")
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



if __name__ == '__main__':
      print("123")
      rospy.init_node('action23')
      print("456")
      #BTAction('action')
      BTAction('action2')
      #BTAction('action3')
      print("789")
      rospy.spin()
