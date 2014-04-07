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


class PerceptionNill(smach.State):
    """
    Perceive the environemt (or not).

    """

    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['succeeded', 'aborted', 'preempted'],
                             input_keys=['view_list'],
                             output_keys=['obj_list'])


    def execute(self, userdata):

        rospy.sleep(2)

        return 'succeeded'


class PerceptionReal (smach.State):

    """
    Perceive the environemt.

    """

    def __init__(self):
        
        smach.State.__init__(self,
                             outcomes=['succeeded', 'aborted', 'preempted'],
                             input_keys=['view_list'],
                             output_keys=['obj_list'])

        self.obj_list = []
        self.active = False
        self.first_call = False

        rospy.Subscriber("/head_xtion/depth/points", PointCloud2, self.callback)


        self.ptu_cmd = rospy.Publisher('/ptu/cmd', JointState)

        rospy.wait_for_service('segment_and_classify')

        try:
            self.obj_rec = rospy.ServiceProxy('segment_and_classify', segment_and_classify )
        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: %s" % e)



    def callback(self,data):

        if self.active == True and self.first_call == True:
            self.first_call = False

            self.pointcloud = data
            
    

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
            
            rospy.loginfo('%i view: receive point cloud',i)
            self.active = True
            self.first_call = True

            # wait for some time to read once from the topic and store it onto self.pointcloud
            # TODO: replace with a service call
            rospy.sleep(2)
            self.active = False

            rospy.loginfo('%i view: call object recognition service',i)
            try:
                obj_rec_resp = self.obj_rec(self.pointcloud)
            except rospy.ServiceException, e:
                rospy.logerr("Service call failed: %s" % e)

            cat_list =  obj_rec_resp.categories_found

            if len(cat_list) == 0:
                rospy.loginfo("%i view: nothing perceived",i)
            else:
                rospy.loginfo('%i view: found categories: %i', i, len(cat_list))
                for cat in cat_list:
                    
                    rospy.loginfo('Categorie: %s', cat.data)
                    obj_desc = dict()
                    obj_desc['type'] = cat.data
                    #obj_desc['probability'] = 1.0
                
                    self.obj_list.append(obj_desc)

            i = i + 1


        # back to original position
        joint_state = JointState()
        joint_state.header.frame_id = 'tessdaf'
        joint_state.name = ['pan', 'tilt']
        joint_state.position = [float(0.0),float(0.0)]
        joint_state.velocity = [float(1.0),float(1.0)]
        joint_state.effort = [float(1.0),float(1.0)]
            
        self.ptu_cmd.publish(joint_state)
            
        userdata.obj_list = self.obj_list

        return 'succeeded'
    
