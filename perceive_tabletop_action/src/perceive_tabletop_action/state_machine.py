#! /usr/bin/env python
import rospy
import smach
import smach_ros
import sys

import actionlib
from actionlib_msgs.msg import *

#from move_base_msgs.msg import *
from strands_navigation_msgs.msg import *

from geometry_msgs.msg import Polygon
from geometry_msgs.msg import Point32
from geometry_msgs.msg import Pose

from perceive_tabletop_action.action_monitor import ActionMonitor
from perceive_tabletop_action.view_planning  import ViewPlanning
#from perceive_tabletop.perception     import *
import perceive_tabletop_action.perception as percept

import numpy
import tf

MOVE_BASE_EXEC_TIMEOUT=rospy.Duration(600.0)
MOVE_BASE_PREEMPT_TIMEOUT=rospy.Duration(10.0)

#callback that build the move_base goal, from the input data        
def move_base_goal_cb(userdata,goal):

    next_goal = strands_navigation_msgs.msg.MonitoredNavigationGoal() # move_base_msgs.msg.MoveBaseGoal()            
    next_goal.target_pose.header.frame_id = "/map"
    next_goal.target_pose.header.stamp = rospy.Time.now()
    next_goal.target_pose.pose = userdata.pose_input

    next_goal.action_server = "move_base"

    return next_goal

class PerceiveTabletopSM(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['succeeded',
                                                    'aborted',
                                                    'preempted'])

        self.userdata.action_completed = False

        self._action_monitor  = ActionMonitor()
        self._view_planning   = ViewPlanning()

        robot = rospy.get_param('robot', 'real')

        if robot == 'real':
            reload (percept)
            self._perception = percept.PerceptionReal()
        elif robot == 'nill':
            self._perception = PerceptionNill()
        else: # 'sim'
            self._perception = PerceptionSim()


        with self:
            smach.StateMachine.add('ActionMonitor', self._action_monitor, 
                                   transitions={'succeeded': 'succeeded',
                                                'action_in_progress':'ViewPlanning',
                                                'aborted':'aborted',
                                                'preempted':'preempted',
                                                'error': 'aborted',},
                                   remapping={'obj_list':'sm_obj_list' #  ,
                                              }) #'action_completed':'sm_action_completed'

            smach.StateMachine.add('ViewPlanning', self._view_planning, 
                                   transitions={'succeeded':'Navigation',
                                                'action_completed':'ActionMonitor',
                                                'aborted':'aborted',
                                                'preempted':'preempted'},
                                   remapping={'obj_list':'sm_obj_list',
                                              'table_pose':'sm_table_pose',
                                              'pose_output':'sm_pose_data',
                                              'view_list':'sm_view_list' #,
                                              }) #                                               'action_completed':'sm_action_completed'

            # The navigation is realized via Move Base directly
            # TODO: replace with monitored navigation in strands_navigation
            smach.StateMachine.add('Navigation',
                                   smach_ros.SimpleActionState('monitored_navigation',
                                                               MonitoredNavigationAction,
                                                               goal_cb = move_base_goal_cb,
                                                               #result_cb = move_base_result_cb,
                                                               input_keys = ['pose_input'],
                                                               exec_timeout = MOVE_BASE_EXEC_TIMEOUT,
                                                               preempt_timeout = MOVE_BASE_PREEMPT_TIMEOUT,
                                                               result_cb=navigation_result_cb,
                                                               outcomes = ['succeeded','aborted','viewpoint_failed','preempted']
                                                               ),
                                   transitions={'succeeded':'Perception',
                                                'aborted':'aborted',
                                                'viewpoint_failed':'ViewPlanning',
                                                'preempted':'preempted'},
                                   remapping={'pose_input':'sm_pose_data'},
                                   )
            smach.StateMachine.add('Perception', self._perception, 
                                   transitions={'succeeded':'ActionMonitor',
                                                'aborted':'aborted',
                                                'preempted':'preempted'},
                                   remapping={'view_list':'sm_view_list',
                                              'obj_list':'sm_obj_list'}
                                   )
def navigation_result_cb(userdata, status, result):
    rospy.loginfo("Monitored navigation result: %i (SUCCEEDED=0, BUMPER_FAILURE=1, LOCAL_PLANNER_FAILURE=2, GLOBAL_PLANNER_FAILURE=3, PREEMPTED=4)", result.sm_outcome)
    if result.sm_outcome == MonitoredNavigationResult.GLOBAL_PLANNER_FAILURE:
        rospy.loginfo("Skipping current viewpoint...")
        return 'viewpoint_failed' 
    
    

    
