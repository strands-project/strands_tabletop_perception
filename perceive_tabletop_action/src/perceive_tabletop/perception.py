#! /usr/bin/env python
import rospy
import smach
import smach_ros
import json

from std_msgs.msg import *
from sensor_msgs.msg import *

from classifier_srv_definitions.srv import segment_and_classify

from world_state.observation import MessageStoreObject, Observation, TransformationStore
from world_state.identification import ObjectIdentification
from world_state.state import World, Object
from world_state.report import PointcloudVisualiser
import world_state.geometry as geometry


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
            self.got_pointcloud = True
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
            self.got_pointcloud = False
            # wait for some time to read once from the topic and store it onto self.pointcloud
            # TODO: replace with a service call

            while not self.got_pointcloud:
                rospy.sleep(0.5)
                rospy.loginfo("Waiting for pointcloud")
            
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
        print "Sleeping for a bit of"
        rospy.sleep(6)

        return 'succeeded'

class PerceptionReal (smach.State):
    """ Perceive the environemt. """

    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['succeeded', 'aborted', 'preempted'],
                             input_keys=['view_list', 'table'],
                             output_keys=['state','obj_list','bbox','cloud', 'table'])

        self.obj_list = []
       
        self.ptu_cmd = rospy.Publisher('/ptu/cmd', JointState)

        rospy.wait_for_service('/classifier_service/segment_and_classify')

        try:
            self.obj_rec = rospy.ServiceProxy('/classifier_service/segment_and_classify', segment_and_classify )
        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: %s" % e)
            
        self._world = World()
        self._pcv = PointcloudVisualiser()

    def execute(self, userdata):
        rospy.loginfo('Executing state %s', self.__class__.__name__)
        rospy.loginfo("Updating info for table %s", userdata.table.name)
        self.obj_list = []

        for i, view in enumerate(userdata.view_list):

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

            userdata.state = 'taking_image'
            
            # mark previous objects on table as finished
            userdata.table.cut_all_children()
            
            # make an observation, kept in the datacentre
            observation =  Observation.make_observation()
            userdata.table.add_observation(observation)
            pointcloud = observation.get_message('/head_xtion/depth/points')
            tf =  TransformationStore.msg_to_transformer(observation.get_message("/tf"))

            rospy.loginfo('%i view: call object recognition service',i)
            userdata.state = 'image_analysis'
            try:
                obj_rec_resp = self.obj_rec(pointcloud)
            except rospy.ServiceException, e:
                rospy.logerr("Service call failed: %s" % e)

            objects = obj_rec_resp.class_results

            depth_to_world = tf.lookupTransform("/map", pointcloud.header.frame_id, 
                                                pointcloud.header.stamp)
            print depth_to_world
            depth_to_world = geometry.Pose(geometry.Point(*(depth_to_world[0])),
                                           geometry.Quaternion(*(depth_to_world[1])))


            world_to_table = userdata.table.pose
            
            self._pcv.clear()
            if len(objects) == 0:
                rospy.loginfo("%i view: nothing perceived",i)
            else:
                rospy.loginfo('%i view: found objects: %i', i, len(objects))

                userdata.cloud = obj_rec_resp.cloud
                userdata.bbox = obj_rec_resp.bbox
                userdata.obj_list = self.obj_list
                for j in range(len(objects)):
                    world_cloud = geometry.transform_PointCloud2(obj_rec_resp.cloud[j],
                                                                 depth_to_world,
                                                                 "/map")
                    self._pcv.add_cloud(world_cloud)
                    
                    new_object =  self._world.create_object()
                    new_object._point_cloud =  MessageStoreObject.create(world_cloud)
                    position = geometry.Point.from_ros_point32(obj_rec_resp.centroid).transform(world_cloud)
                    new_object.add_pose(geometry.Pose(position)) # no orientation
                    userdata.table.add_child(new_object)

                    
                    obj = objects[j]
                    max_idx = obj.confidence.index(max(obj.confidence))
                    obj_desc = dict()
                    obj_desc['type'] = obj.class_type[max_idx].data.strip('/')
                    
                    rospy.loginfo('Object: %s', obj_desc['type'])
                    self.obj_list.append(obj_desc)
            self._pcv.publish()

        return 'succeeded'

