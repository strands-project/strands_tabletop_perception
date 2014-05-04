#! /usr/bin/env python
import rospy
import smach
import smach_ros

from ros_datacentre.message_store import MessageStoreProxy
from strands_perception_msgs.msg import Table

from geometry_msgs.msg import Polygon
from geometry_msgs.msg import Point32
from geometry_msgs.msg import Pose

from sensor_msgs.msg import JointState

import numpy
import tf


class ActionMonitor(smach.State):

    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['succeeded','action_in_progress', 'aborted', 'preempted'],
                             input_keys=['action_completed', 'table_id'],
                             output_keys=['action_completed', 'table_area'],
                             )

        self.ptu_cmd = rospy.Publisher('/ptu/cmd', JointState)
        self.got_table = False
                                                  
    def execute(self, userdata):

        # back to original position
        joint_state = JointState()
        joint_state.header.frame_id = 'tessdaf'
        joint_state.name = ['pan', 'tilt']
        joint_state.position = [float(0.0),float(0.0)]
        joint_state.velocity = [float(1.0),float(1.0)]
        joint_state.effort = [float(1.0),float(1.0)]
            
        self.ptu_cmd.publish(joint_state)


        if self.got_table == False:

            rospy.loginfo('Retrieving polygon for table: %s', userdata.table_id)
        

            self._msg_store=MessageStoreProxy(collection="tables")
            table = self._get_table(str(userdata.table_id))  #userdata.table_id) #TODO: what if non exist?
            # o = table.pose.pose.orientation
            # quat=numpy.array([o.x,o.y,o.z,o.w])
            # mat=tf.transformations.quaternion_matrix(quat)
            # mat[0][3]=table.pose.pose.position.x
            # mat[1][3]=table.pose.pose.position.y
            # mat[2][3]=table.pose.pose.position.z

            # for pt in table.tabletop.points:
            #     p=numpy.array([pt.x, pt.y, pt.z,1])
            #     new=numpy.dot(mat,p)[:-1]
            #     pt.x=new[0]
            #     pt.y=new[1]
            #     pt.z=new[2]
        
            poly = table.tabletop #Polygon(points) #

            userdata.table_area = table.tabletop
            self.got_table = True
            

        if userdata.action_completed == True:
            rospy.loginfo('Action completed')
            userdata.action_completed = False
            return 'succeeded'

        rospy.loginfo('Action in progress')
        return 'action_in_progress'
            
    def _get_table(self, table_id):
        """ Get a specific table """
        query = {}
        query["table_id"] = table_id
        #TODO: what if does not exist....
        return self._msg_store.query(Table._type, query,single=True)[0]
                                                  
