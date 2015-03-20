#! /usr/bin/env python
import rospy
import smach
import smach_ros
import json

from actionlib import *
from actionlib.msg import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import threading
from scitos_ptu.msg import PtuGotoAction,PtuGotoGoal
import math

from std_msgs.msg import *
from sensor_msgs.msg import *

from viper.core.view import View

class GoTo(smach.State):
    """
    GoTo action

    """

    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['succeeded', 'aborted', 'preempted'],
                             input_keys=['robot_pose','ptu_state'])

        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Wait for monitored navigation server")
        self.client.wait_for_server(rospy.Duration(60))
        self.ptu_client = actionlib.SimpleActionClient('SetPTUState', PtuGotoAction)
        rospy.loginfo("Wait for PTU action server")
        self.ptu_client.wait_for_server(rospy.Duration(60))

    def execute(self, userdata):
        rospy.loginfo('Executing state %s', self.__class__.__name__)

        self.robot_pose = userdata.robot_pose  

        self.mb_done = False
        mb_thread = threading.Thread(target = self.move_base)
        mb_thread.start()

        ptu_state = userdata.ptu_state

        goal = PtuGotoGoal()
        goal.pan = ptu_state.position[ptu_state.name.index('pan')] * 180/math.pi
        goal.tilt = ptu_state.position[ptu_state.name.index('tilt')]  * 180/math.pi
        goal.pan_vel = ptu_state.velocity[ptu_state.name.index('pan')] * 100
        goal.tilt_vel = ptu_state.velocity[ptu_state.name.index('tilt')] * 100
        self.ptu_client.send_goal(goal)
        self.ptu_client.wait_for_result()

        rospy.loginfo("Reached ptu goal")
        while self.mb_done == False: #self.client.get_state() == GoalStatus.ACTIVE:
            rospy.sleep(rospy.Duration(0.5))
            rospy.loginfo("Wait for move_base")
        rospy.loginfo("Reached move_base goal")
        self.client.cancel_goal()

        return 'succeeded'

    def mb_done_cb(self,status,result):
        self.mb_done = True
    
    def move_base(self):
        self.client.wait_for_server(rospy.Duration(60))
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = self.robot_pose
        self.client.send_goal(goal, done_cb=self.mb_done_cb)
        self.client.wait_for_result()

