#! /usr/bin/env python
import rospy
import smach
import smach_ros

import actionlib
from actionlib_msgs.msg import *
from move_base_msgs.msg import *

from geometry_msgs.msg import Polygon
from geometry_msgs.msg import Point32
from geometry_msgs.msg import Pose

from perceive_tabletop.action_monitor import ActionMonitor
from perceive_tabletop.view_planning  import ViewPlanning
from perceive_tabletop.perception     import PerceptionSim

from ros_datacentre.message_store import MessageStoreProxy
from strands_perception_msgs.msg import Table

import tf
import numpy

MOVE_BASE_EXEC_TIMEOUT=rospy.Duration(600.0)
MOVE_BASE_PREEMPT_TIMEOUT=rospy.Duration(10.0)

#callback that build the move_base goal, from the input data        
def move_base_goal_cb(userdata,goal):
    
    next_goal = move_base_msgs.msg.MoveBaseGoal()            
    next_goal.target_pose.header.frame_id = "/map"
    next_goal.target_pose.header.stamp = rospy.Time.now()
    next_goal.target_pose.pose = userdata.pose_input 
    
    return next_goal

def _get_table(table_id):
    """ Get a specific table """
    # UNUSED.
    query = {}
    query["table_id"] = table_id
    #TODO: what if does not exist....
    return self._msg_store.query(Table._type, {}, query)[0]


class PerceiveTabletopSM(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['succeeded',
                                                    'aborted',
                                                    'preempted'],
                                                    input_keys=['goal'])

        

        #polygon = [[-1.0,0.0],[4.0,0.0],[4.0,-4.0],[-1.0,-4.0]]

        self._msg_store=MessageStoreProxy()
        table=self._get_table(self.userdata.goal) #"lab_test_1")
        mat=tf.transformations.quaternion_matrix(table.pose.pose.orientation)
        mat[3][0]=table.pose.pose.position.x
        mat[3][1]=table.pose.pose.position.y
        mat[3][2]=table.pose.pose.position.z

        for pt in table.tabletop.points:
            p=numpy.array([pt.x, pt.y, pt.z,1])
            new=numpy.dot(mat,p)[:-1]
            pt.x=new[0]
            pt.y=new[1]
            pt.z=new[2]

        # table in TUM kitchen in map frame coordinates
        #polygon = [[0.8,-1.5],[1.4,-1.5],[1.4,-3.7],[0.8,-3.7]]
        
        rospy.loginfo('Polygon: %s', polygon)
        points = []
        for point in polygon:
            rospy.loginfo('Point: %s', point)
            points.append(Point32(float(point[0]),float(point[1]),0))

        poly = table.tabletop #Polygon(points)
        self.userdata.sm_table_area = poly
        self.userdata.sm_table_pose = []

        self.userdata.sm_action_completed = False

        self._action_monitor  = ActionMonitor()
        self._view_planning   = ViewPlanning()

        robot = rospy.get_param('robot', 'sim')

        if robot == 'real':
            self._perception = PerceptionReal()
        else: # 'sim'
            self._perception = PerceptionSim()

        
        with self:
             smach.StateMachine.add('ActionMonitor', self._action_monitor, 
                                    transitions={'succeeded': 'succeeded',
                                                 'action_in_progress':'ViewPlanning',
                                                 'aborted':'aborted',
                                                 'preempted':'preempted'},
                                    remapping={'obj_list':'sm_obj_list',
                                               'action_completed':'sm_action_completed'})

             smach.StateMachine.add('ViewPlanning', self._view_planning, 
                                    transitions={'succeeded':'Navigation',
                                                 'action_completed':'ActionMonitor',
                                                 'aborted':'aborted',
                                                 'preempted':'preempted'},
                                    remapping={'obj_list':'sm_obj_list',
                                               'table_pose':'sm_table_pose',
                                               'table_area':'sm_table_area',
                                               'pose_output':'sm_pose_data',
                                               'view_list':'sm_view_list',
                                               'action_completed':'sm_action_completed'})
             
             # The navigation is realized via Move Base directly
             # TODO: replace with monitored navigation in strands_navigation
             smach.StateMachine.add('Navigation',
                                    smach_ros.SimpleActionState('move_base',
                                                                MoveBaseAction,
                                                                goal_cb = move_base_goal_cb,
                                                                #result_cb = move_base_result_cb,
                                                                input_keys = ['pose_input'],
                                                                exec_timeout = MOVE_BASE_EXEC_TIMEOUT,
                                                                preempt_timeout = MOVE_BASE_PREEMPT_TIMEOUT
                                                                ),
                                    transitions={'succeeded':'Perception',
                                                 'aborted':'aborted',
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


    def _get_table(self, table_id):
        """ Get a specific table """
        # UNUSED.
        query = {}
        query["table_id"] = table_id
        #TODO: what if does not exist....
        return self._msg_store.query(Table._type, {}, query)[0]

    
