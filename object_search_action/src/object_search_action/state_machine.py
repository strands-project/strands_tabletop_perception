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
        
        perception = rospy.get_param('perception', 'real')
        if perception == 'real':# 'real':
            reload (percept)
            self._perception = percept.PerceptionReal()
        else: # perception == 'nill':
            self._perception = percept.PerceptionNill()
        # else: # 'sim'
        #     self._perception = PerceptionSim()


        with self:
            smach.StateMachine.add('Setup', self._setup, 
                                   transitions={'succeeded': 'ViewPlanning',
                                                'aborted':'Shutdown',
                                                'preempted':'Shutdown'})

            smach.StateMachine.add('ViewPlanning', self._view_planning, 
                                   transitions={'succeeded': 'Executive',
                                                'aborted':'Shutdown',
                                                'preempted':'Shutdown'})

            smach.StateMachine.add('Executive', self._executive, 
                                   transitions={'succeeded': 'GoTo', 
                                                'no_views': 'Shutdown',
                                                'aborted':'Shutdown',
                                                'preempted':'Shutdown'})

            smach.StateMachine.add('GoTo', self._goto, 
                                   transitions={'succeeded': 'Perception',
                                                'aborted':'Executive',
                                                'preempted':'Shutdown'})

            smach.StateMachine.add('Perception', self._perception, 
                                   transitions={'succeeded':'Executive',
                                                'found_all_objects':'Shutdown',
                                                'aborted':'Shutdown',
                                                'preempted':'Shutdown'}
                               )

            smach.StateMachine.add('Shutdown', self._shutdown, 
                                   transitions={'succeeded':'succeeded',
                                                'aborted':'aborted',
                                                'preempted':'preempted'}
                               )
                        
            

    
