#! /usr/bin/env python
import roslib; roslib.load_manifest('perceive_tabletop_action')
import rospy

import threading

import actionlib

#from smach_ros import ActionServerWrapper

from  perceive_tabletop_action.msg import *
from  perceive_tabletop.state_machine import PerceiveTabletopSM

_EPSILON = 0.0001

class PerceiveTabletopActionServer:


    _feedback = perceive_tabletop_action.msg.PerceiveTabletopFeedback()
    _result   = perceive_tabletop_action.msg.PerceiveTabletopResult()
    
    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name,
                                                perceive_tabletop_action.msg.PerceiveTabletopAction,
                                                execute_cb=self.execute_cb,
                                                auto_start = False)

        self._as.start()
        rospy.loginfo('Started action server for tabletop perception')
        
        
    def execute_cb(self, goal):

        rospy.loginfo('Received request: %s', goal.table_id)
        
        # helper variables
        r = rospy.Rate(1)
        success = True
        
        # create the state machine
        sm = PerceiveTabletopSM()

        sm.userdata.current_view = 0
        sm.userdata.num_of_views = 0
        sm.userdata.table_id = goal.table_id

        smach_thread = threading.Thread(target = sm.execute)
        smach_thread.start()

        #outcome = self.agent.execute_sm_with_introspection()
        r.sleep()
        
        while sm.is_running() and not sm.preempt_requested():
            # check that preempt has not been requested by the client
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._as.set_preempted()
                sm.request_preempt()
                success = False
                break

            #rospy.loginfo(self.agent.get_sm().get_active_states())
            userdata = sm.userdata

            
                
            # get current pose form state machine
            self._feedback = perceive_tabletop_action.msg.PerceiveTabletopFeedback()

            if userdata.num_of_views == 0:
                self._feedback.percent_complete = 0
            else:
                self._feedback.percent_complete = (float(userdata.current_view) / float(userdata.num_of_views)) * 100 

            self._as.publish_feedback(self._feedback)

            r.sleep()

        if success:
            
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)
        else:
            rospy.loginfo('%s: Failed' % self._action_name)



if __name__ == '__main__':
    rospy.init_node('perceive_tabletop_server')
    ptas = PerceiveTabletopActionServer('perceive_tabletop')
    rospy.spin()
    


    
#     def __init__(self, server_name):

#         self.server_name = server_name
#         self.pta_sm = PerceiveTabletopSM()
        
        
#     def run_server(self):

#         # Construct action server wrapper
#         asw = ActionServerWrapper(
#             self.server_name, PerceiveTabletopAction,
#             wrapped_container = self.pta_sm,
#             succeeded_outcomes = ['succeeded'],
#             aborted_outcomes = ['aborted'],
#             preempted_outcomes = ['preempted'],
#             goal_key = 'table_id')

#         # Run the server in a background thread
#         asw.run_server()

#         # Wait for control-c
#         rospy.spin()
        

# if __name__ == '__main__':
#   rospy.init_node('perceive_tabletop_server')
#   ptas = PerceiveTabletopActionServer('perceive_tabletop')
#   ptas.run_server()
