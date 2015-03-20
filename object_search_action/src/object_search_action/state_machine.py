#! /usr/bin/env python
import rospy
import smach
import smach_ros
import sys

import actionlib
from actionlib_msgs.msg import *

##from move_base_msgs.msg import *
#from strands_navigation_msgs.msg import *

#from geometry_msgs.msg import Polygon
#from geometry_msgs.msg import Point32
#from geometry_msgs.msg import Pose

from object_search_action.setup import Setup
from object_search_action.view_planning  import ViewPlanning
from object_search_action.executive import Executive
from object_search_action.navigation import GoTo
from object_search_action.shutdown import Shutdown

import object_search_action.perception as percept

import numpy
import tf

class ObjectSearchSM(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['succeeded',
                                                    'aborted',
                                                    'preempted'])

        self.userdata.action_completed = False

        self._setup  = Setup()
        self._view_planning   = ViewPlanning()
        self._executive       = Executive()
        self._goto            = GoTo()
        self._shutdown        = Shutdown()
        
        robot = rospy.get_param('robot', 'nill')
        if robot == 'real':
            reload (percept)
            self._perception = percept.PerceptionReal()
        else: # robot == 'nill':
            self._perception = percept.PerceptionNill()
        # else: # 'sim'
        #     self._perception = PerceptionSim()


        with self:
            smach.StateMachine.add('Setup', self._setup, 
                                   transitions={'succeeded': 'ViewPlanning',
                                                'aborted':'aborted',
                                                'preempted':'preempted'})

            smach.StateMachine.add('ViewPlanning', self._view_planning, 
                                   transitions={'succeeded': 'Executive',
                                                'aborted':'aborted',
                                                'preempted':'preempted'})

            smach.StateMachine.add('Executive', self._executive, 
                                   transitions={'succeeded': 'GoTo',
                                                'no_views': 'Shutdown',
                                                'aborted':'aborted',
                                                'preempted':'preempted'})

            smach.StateMachine.add('GoTo', self._goto, 
                                   transitions={'succeeded': 'Perception',
                                                'aborted':'',
                                                'preempted':'preempted'})

            smach.StateMachine.add('Perception', self._perception, 
                                   transitions={'succeeded':'Executive',
                                                'aborted':'aborted',
                                                'preempted':'preempted'}
                               )

            smach.StateMachine.add('Shutdown', self._shutdown, 
                                   transitions={'succeeded':'succeeded',
                                                'aborted':'aborted',
                                                'preempted':'preempted'}
                               )
                        
            
            # # The navigation is realized via Move Base directly
            # # TODO: replace with monitored navigation in strands_navigation
            # smach.StateMachine.add('Navigation',
            #                        smach_ros.SimpleActionState('monitored_navigation',
            #                                                    MonitoredNavigationAction,
            #                                                    goal_cb = move_base_goal_cb,
            #                                                    #result_cb = move_base_result_cb,
            #                                                    input_keys = ['pose_input'],
            #                                                    exec_timeout = MOVE_BASE_EXEC_TIMEOUT,
            #                                                    preempt_timeout = MOVE_BASE_PREEMPT_TIMEOUT,
            #                                                    result_cb=navigation_result_cb,
            #                                                    outcomes = ['succeeded','aborted','viewpoint_failed','preempted']
            #                                                    ),
            #                        transitions={'succeeded':'Perception',
            #                                     'aborted':'aborted',
            #                                     'viewpoint_failed':'ViewPlanning',
            #                                     'preempted':'preempted'},
            #                        remapping={'pose_input':'sm_pose_data'},
            #                        )

# def navigation_result_cb(userdata, status, result):
#     rospy.loginfo("Monitored navigation result: %i (SUCCEEDED=0, BUMPER_FAILURE=1, LOCAL_PLANNER_FAILURE=2, GLOBAL_PLANNER_FAILURE=3, PREEMPTED=4)", result.sm_outcome)
#     if result.sm_outcome == MonitoredNavigationResult.GLOBAL_PLANNER_FAILURE:
#         rospy.loginfo("Skipping current viewpoint...")
#         return 'viewpoint_failed' 
    
    

    
