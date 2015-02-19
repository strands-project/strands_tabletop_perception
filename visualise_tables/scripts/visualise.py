#!/usr/bin/env python
import roslib; roslib.load_manifest("visualise_tables")
import rospy
import sys
import copy
import numpy
import tf

from interactive_markers.interactive_marker_server import (
    InteractiveMarker,
    InteractiveMarkerServer,
     )
from visualization_msgs.msg import Marker, InteractiveMarkerControl
from strands_perception_msgs.msg import Table
from mongodb_store.message_store import MessageStoreProxy
from geometry_msgs.msg import Point32, Quaternion
        
class Visualiser(object):
    def __init__(self, interactive=False):
        rospy.init_node("tables_visualiser", anonymous=True)

        self._marker_server = InteractiveMarkerServer("table_markers")
        self._msg_store=MessageStoreProxy(collection="tables")
        self._tables = self._get_tables()
        self._interactive = interactive

        for table in self._tables:
            table_cp=copy.deepcopy(table)
            self._create_marker(table_cp,table.table_id)

        rospy.loginfo("Started table visualiser, " + str(len(self._tables)) +
                      " tables in datacentre")
        rospy.spin()

        T=Table()
        T.table_id="random_nothingness"
        T.header.frame_id="/map"
        for p in [(0,0,0), (0,1.795,0),(0.84,1.795,0),(0.84,0,0),(0,0,0)]:
            pt=Point32()
            pt.x=p[0]
            pt.y=p[1]
            pt.z=p[2]
            T.tabletop.points.append(pt)
#        self._msg_store.insert(T)
        
    def _get_tables(self):
        """ Get a list of tables """
        table_list = self._msg_store.query(Table._type)
        return zip(*table_list)[0]
    
    def _get_table(self, table_id):
        """ Get a specific table """
        # UNUSED.
        query = {}
        query["table_id"] = table_id
        #TODO: what if does not exist....
        return self._msg_store.query(Table._type, {}, query)[0]

    def _create_marker(self, table,
                       marker_description="table interactive marker"):
        assert isinstance(table, Table)
        table.tabletop.points.append(copy.deepcopy(table.tabletop.points[0]))
        # create an interactive marker for our server
        marker = InteractiveMarker()
        marker.header.frame_id = table.header.frame_id
        marker.name = table.table_id
        print table.table_id
        marker.description = marker_description

        # the marker in the middle
        box_marker = Marker()
        box_marker.type = Marker.LINE_STRIP
        box_marker.scale.x = 0.1
        box_marker.color.r = 1.0
        box_marker.color.g = 1.0
        box_marker.color.b = 0.5
        box_marker.color.a = 0.8
        
        # Move the world-frame table points into table frame.
        o = table.pose.pose.orientation
        quat=numpy.array([o.x,o.y,o.z,o.w])
        mat=tf.transformations.quaternion_matrix(quat)
        mat[0][3]= table.pose.pose.position.x
        mat[1][3]= table.pose.pose.position.y
        mat[2][3]= table.pose.pose.position.z
        mat = numpy.linalg.inv(mat)

        for pt in table.tabletop.points:
            p=numpy.array([pt.x, pt.y, pt.z,1])
            new=numpy.dot(mat,p)[:-1]
            pt.x=new[0]
            pt.y=new[1]
            pt.z=new[2]
          
            box_marker.points.append(pt)

        # create a non-interactive control which contains the box
        box_control = InteractiveMarkerControl()
        box_control.always_visible = True
        box_control.markers.append( box_marker )
        marker.controls.append( box_control )

        def add_control(orientation,name, move_rotate=1):
            control = InteractiveMarkerControl()
            control.orientation = orientation
            control.name = name
            if move_rotate==1:
                control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
            else:
                control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
            marker.controls.append(control)
            
        if self._interactive:
            add_control(Quaternion(1,0,0,1),"move_x",1)
            add_control(Quaternion(0,1,0,1),"move_y",1)
            add_control(Quaternion(0,0,1,1),"move_z",1)
            add_control(Quaternion(1,0,0,1),"rotate_x",2)
            add_control(Quaternion(0,1,0,1),"rotate_y",2)
            add_control(Quaternion(0,0,1,1),"rotate_z",2)

        self._marker_server.insert(marker, self._marker_feedback)
        self._marker_server.applyChanges()
        
	self._marker_server.setPose( marker.name, table.pose.pose )
        self._marker_server.applyChanges()


    def _marker_feedback(self, feedback):
        table = [t for t in self._tables if t.table_id==feedback.marker_name]
        assert len(table)==1
        table = table[0]
        table.pose.pose = feedback.pose
        #TODO: Set covariance?
        self._msg_store.update(table,
                               message_query={"table_id": feedback.marker_name},
                               upsert=True);

        
if __name__=="__main__":
    if len(sys.argv)>1 and sys.argv[1]=="edit":
        print "Edit mode"
        visualiser = Visualiser(True)
    else:
        visualiser = Visualiser(False)
