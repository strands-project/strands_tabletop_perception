#! /usr/bin/env python

import roslib; roslib.load_manifest('perceive_tabletop_action')
import rospy
import actionlib

from perceive_tabletop_action.msg import *

class PerceiveTabletopServer:

    # create messages that are used to publish feedback/result
    _feedback = PerceiveTabletopFeedback()
    _result   = PerceiveTabletopResult()

    def __init__(self, name):
        self._action_name = name    
        self._as = actionlib.SimpleActionServer(self._action_name, PerceiveTabletopAction, self.execute_cb, False)
        self._as.start()

    def execute_cb(self, goal):

        rospy.loginfo('%s: Received goal: %s' % (self._action_name, goal.table_id))
        
        success = True
        
        # Do lots of awesome groundbreaking robot stuff here
        
        #if self._as.is_preempt_requested():
        #    rospy.loginfo('%s: Preempted' % self._action_name)
        #    self._as.set_preempted()
        #    success = False
            
        # publish the feedback
        #self._as.publish_feedback(self._feedback)

        if success:
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)
   

if __name__ == '__main__':
  rospy.init_node('perceive_tabletop_server')
  server = PerceiveTabletopServer('perceive_tabletop')
  rospy.spin()
