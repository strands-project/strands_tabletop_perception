#! /usr/bin/env python
import rospy
import smach
import smach_ros
import json

from std_msgs.msg import *
from sensor_msgs.msg import *

from recognition_srv_definitions.srv import recognize, recognizeResponse, recognizeRequest

# from world_state.observation import MessageStoreObject, Observation, TransformationStore
# from world_state.identification import ObjectIdentification
# from world_state.state import World, Object
# from world_state.report import PointCloudVisualiser, create_robblog
# import world_state.geometry as geometry

import numpy as np


class PerceptionNill(smach.State):
    """
    Perceive the environemt (or not).

    """

    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['succeeded', 'aborted', 'preempted'],
                             input_keys=[],
                             output_keys=[])

    def execute(self, userdata):
        rospy.loginfo("Sleeping for a bit")
        rospy.sleep(3)
        return 'succeeded'

class PerceptionReal (smach.State):
    """ Perceive the environemt. """

    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['succeeded', 'aborted', 'preempted'],
                             input_keys=[],
                             output_keys=[])

        self.pc_frame = rospy.get_param('~camera', '/head_xtion/depth/points')
        self.obj_list = []

        self.ir_service_name = '/recognition_service/mp_recognition'
        rospy.loginfo('Wait for service %s', self.ir_service_name)
        rospy.wait_for_service(self.ir_service_name)

        try:
            self.ir_service = rospy.ServiceProxy(self.ir_service_name, recognize)
        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: %s" % e)
            
        #self._world = World()
        #self._pcv = PointCloudVisualiser()

    def execute(self, userdata):
        rospy.loginfo('Executing state %s', self.__class__.__name__)
        self.obj_list = []

        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'


        # get point cloud
        try:
            rospy.loginfo('Waiting for pointcloud')
            pointcloud = rospy.wait_for_message(self.pc_frame, PointCloud2 , timeout=60.0)
            rospy.loginfo('Got pointcloud')
        except rospy.ROSException, e:
            rospy.logwarn("Failed to get %s" % self.pc_frame)
            return 'aborted'

        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'

        try:
            req = recognizeRequest()
            req.cloud = pointcloud
            req.complex_result.data = True
            rospy.loginfo('Calling service %s' %  self.ir_service_name)
            res = self.ir_service(req)
            rospy.loginfo('Received result from %s' % self.ir_service_name)
        except rospy.ServiceException, e2:
            rospy.loginfo("Service call failed: %s", e2)
            return 'aborted'

        for i in range(len(res.ids)):
            rospy.loginfo("Recognized: %s %s" % (res.ids[i], res.confidence[i]))
            self.obj_list.append(res.ids[i])
            
        return 'succeeded'

