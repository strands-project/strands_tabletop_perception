#! /usr/bin/env python
import rospy
import smach
import smach_ros
import json

from std_msgs.msg import *
from sensor_msgs.msg import *

class PerceptionSim(smach.State):
    """
    Perceive the environemt.

    """

    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['succeeded', 'aborted', 'preempted'],
                             input_keys=['view_list'],
                             output_keys=['obj_list'])

        rospy.Subscriber("semcam", String, self.callback)
        self.ptu_cmd = rospy.Publisher('/ptu/cmd', JointState)
        
        self.obj_list = []
        self.active = False
        self.first_call = False

    def callback(self,data):
        if self.active == True and self.first_call == True:
            self.first_call = False
            obj_list = json.loads(data.data)
            if len(obj_list) == 0:
                rospy.loginfo("Nothing perceived")
            for obj_desc in obj_list:
                rospy.loginfo("Perceived: %s" % obj_desc.get('name'))

            for obj in obj_list:
                self.obj_list.append(obj)

    def execute(self, userdata):

        rospy.loginfo('Executing state %s', self.__class__.__name__)

        self.obj_list = []

        i = 1
        for view in userdata.view_list:

            rospy.loginfo('%i view: set PTU to %s',i,view)

            joint_state = JointState()
            joint_state.header.frame_id = 'tessdaf'
            joint_state.name = ['pan', 'tilt']
            joint_state.position = [float(view[0]),float(view[1])]
            joint_state.velocity = [float(1.0),float(1.0)]
            joint_state.effort = [float(1.0),float(1.0)]
            
            self.ptu_cmd.publish(joint_state)
            
            # wait until PTU has finished or point cloud get screwed up
            rospy.sleep(2)
            
            rospy.loginfo('%i view: receive from semantic camera',i)
            self.active = True
            self.first_call = True

            # wait for some time to read once from the topic and store it onto self.pointcloud
            # TODO: replace with a service call
            rospy.sleep(2)
            
            self.active = False

            i = i + 1


        userdata.obj_list = self.obj_list

        # back to original position
        joint_state = JointState()
        joint_state.header.frame_id = 'tessdaf'
        joint_state.name = ['pan', 'tilt']
        joint_state.position = [float(0.0),float(0.0)]
        joint_state.velocity = [float(1.0),float(1.0)]
        joint_state.effort = [float(1.0),float(1.0)]
            
        self.ptu_cmd.publish(joint_state)

        return 'succeeded'

