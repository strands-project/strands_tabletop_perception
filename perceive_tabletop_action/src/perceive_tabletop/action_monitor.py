#! /usr/bin/env python
import rospy
import smach
import smach_ros

from ros_datacentre.message_store import MessageStoreProxy
from strands_perception_msgs.msg import Table

from geometry_msgs.msg import Polygon
from geometry_msgs.msg import Point32
from geometry_msgs.msg import Pose

import numpy
import tf


class ActionMonitor(smach.State):

    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['succeeded','action_in_progress', 'aborted', 'preempted'],
                             input_keys=['action_completed', 'table_id'],
                             output_keys=['action_completed', 'sm_table_area'],
                             )
                                                  
    def execute(self, userdata):

        rospy.loginfo('TABLE: %s', userdata.table_id)
        

        self._msg_store=MessageStoreProxy()
        table = self._get_table(str(userdata.table_id))  #userdata.table_id) #TODO: what if non exist?
        o = table.pose.pose.orientation
        quat=numpy.array([o.x,o.y,o.z,o.w])
        mat=tf.transformations.quaternion_matrix(quat)
        mat[0][3]=table.pose.pose.position.x
        mat[1][3]=table.pose.pose.position.y
        mat[2][3]=table.pose.pose.position.z

        for pt in table.tabletop.points:
            p=numpy.array([pt.x, pt.y, pt.z,1])
            new=numpy.dot(mat,p)[:-1]
            pt.x=new[0]
            pt.y=new[1]
            pt.z=new[2]
        
        # Table in bham lab
        # polygon = [[3.11,2.12], [5.09,2.12], [5.09,3.3], [3.11, 3.3]]
        # #polygon = [[-1.9,-5.2],[-1.9,-5.8],[-3.8,-5.8],[-3.8,-5.2]]
        # rospy.loginfo('Polygon: %s', polygon)
        # points = []
        # for point in polygon:
        #     #rospy.loginfo('Point: %s', point)
        #     points.append(Point32(float(point[0]),float(point[1]),0))

        poly = table.tabletop #Polygon(points) #
        #poly = Polygon(points) 
        #rospy.loginfo("Table: %s" % poly)

        userdata.sm_table_area = poly
        
        if userdata.action_completed == True:
            userdata.action_completed = False
            return 'succeeded'

        return 'action_in_progress'
        

    
        # action_server_name=userdata.goal.action_server
        # action_client= actionlib.SimpleActionClient(action_server_name, MoveBaseAction)
        # action_client.wait_for_server()
        # action_client.send_goal(userdata.goal)
        # status= action_client.get_state()
        # while status==GoalStatus.PENDING or status==GoalStatus.ACTIVE:   
        #     status= action_client.get_state()
        #     if self.preempt_requested():
        #         action_client.cancel_goal()
        #         self.service_preempt()
        #     action_client.wait_for_result(rospy.Duration(0.2))
        
        # if status == GoalStatus.SUCCEEDED:
        #     userdata.n_nav_fails = 0
        #     return 'succeeded'
        # elif status==GoalStatus.PREEMPTED:
        #     return 'preempted'
        # else:
        #     userdata.n_nav_fails = userdata.n_nav_fails + 1
        #     return 'aborted'
    
    def _get_table(self, table_id):
        """ Get a specific table """
        query = {}
        query["table_id"] = table_id
        #TODO: what if does not exist....
        return self._msg_store.query(Table._type, query,single=True)[0]
                                                  
