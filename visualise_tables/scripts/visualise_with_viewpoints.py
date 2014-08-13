#!/usr/bin/env python
import roslib; roslib.load_manifest("visualise_tables")
import rospy
import sys
import copy
import numpy
import tf
from math import sqrt
from threading import Timer

from interactive_markers.interactive_marker_server import (
    InteractiveMarker,
    InteractiveMarkerServer,
     )
from visualization_msgs.msg import Marker, InteractiveMarkerControl
from strands_perception_msgs.msg import Table
from mongodb_store.message_store import MessageStoreProxy
from geometry_msgs.msg import Point32, Quaternion, PoseStamped, Pose
        
def mat_to_quat(m):
    """ http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/
    takes a numpy matrix and gives ROS quaternion"""
    tr = m[0, 0] + m[1, 1] + m[2, 2]

    if (tr > 0): 
        S = sqrt(tr+1.0) * 2
        qw = 0.25 * S
        qx = (m[2, 1] - m[1, 2]) / S
        qy = (m[0, 2] - m[2, 0]) / S 
        qz = (m[1, 0] - m[0, 1]) / S 
    elif ((m[0,0] > m[1,1])&(m[0,0] > m[2,2])):
        S = sqrt(1.0 + m[0,0] - m[1,1] - m[2,2]) * 2
        qw = (m[2,1] - m[1,2]) / S
        qx = 0.25 * S
        qy = (m[0,1] + m[1,0]) / S
        qz = (m[0,2] + m[2,0]) / S
    elif (m[1,1] > m[2,2]):
        S = sqrt(1.0 + m[1,1] - m[0,0] - m[2,2]) * 2; 
        qw = (m[0,2] - m[2,0]) / S;
        qx = (m[0,1] + m[1,0]) / S; 
        qy = 0.25 * S;
        qz = (m[1,2] + m[2,1]) / S; 
    else:
        S = sqrt(1.0 + m[2,2] - m[0,0] - m[1,1]) * 2; 
        qw = (m[1,0] - m[0,1]) / S;
        qx = (m[0,2] + m[2,0]) / S;
        qy = (m[1,2] + m[2,1]) / S;
        qz = 0.25 * S;
        
    quat =  Quaternion()
    quat.x = qx
    quat.y = qy
    quat.z = qz
    quat.w = qw
    return quat
        
class Visualiser(object):
    def __init__(self, interactive=False):
        rospy.init_node("tables_visualiser", anonymous=True)

        self._marker_server = InteractiveMarkerServer("table_markers")
        self._msg_store=MessageStoreProxy(collection="frozen_tables")
        self._tables = self._get_tables()
        self._viewpoints = self._get_viewpoints()
        self._interactive = interactive

        for table, table_meta in self._tables:
            table_cp=copy.deepcopy(table)
            self._create_marker(table_cp,table.table_id)
            
            # Check a viewpoint exists for this table
            for pt, meta in self._viewpoints:
                if meta["name"] == table.table_id + ".viewpoint":
                    # found it
                    break
            else:
                # Create a new viewpoint for this unknown or new table
                rospy.loginfo("Creating viewpoint object for %s"%table.table_id)
                p = Pose()
                self._msg_store.insert_named(table.table_id+".viewpoint", p)

        self._viewpoints = self._get_viewpoints()
        
        for viewpoint, vpmeta in self._viewpoints:
            for tbl, meta in self._tables: # uggly review hack code.
                if meta["name"] == vpmeta['name'][:-10]:
                    # found it
                    rospy.loginfo("%s for %s"%(vpmeta['name'], meta['name']))
                    break
            else:
                rospy.logerr("No table for view point %s"%vpmeta['name'])
                sys.exit(1)
            view_cp=copy.deepcopy(viewpoint)
            self._create_viewpoint_marker(tbl, view_cp,vpmeta["name"],
                                          marker_description="")

        rospy.loginfo("Started table visualiser, " + str(len(self._tables)) +
                      " tables in datacentre")
        rospy.spin()
        
    def _get_tables(self):
        """ Get a list of tables """
        table_list = self._msg_store.query(Table._type)
        return table_list
    
    def _get_viewpoints(self):
        """ Get a list of tables """
        table_list = self._msg_store.query(Pose._type)
        return table_list

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
        
    def _create_viewpoint_marker(self, table, viewpoint, name, 
                       marker_description="viewpoint interactive marker"):

        # create an interactive marker for our server
        marker = InteractiveMarker()
        marker.header.frame_id = "/map"
        marker.name = name
        marker.scale = 0.2
        marker.description = marker_description

        # the marker in the middle
        box_marker = Marker()
        box_marker.type = Marker.MESH_RESOURCE
        box_marker.scale.x = 1
        box_marker.scale.y = 1
        box_marker.scale.z = 1
        box_marker.color.r = 1.0
        box_marker.color.g = 0.8
        box_marker.color.b = 0.8
        box_marker.color.a = 1
        box_marker.pose.position.z = -1.3 
        box_marker.pose.position.x = 0.3 
        box_marker.mesh_resource = "package://visualise_tables/meshes/person.dae";
        
        # Move pose into /world.
        o = table.pose.pose.orientation
        quat=numpy.array([o.x,o.y,o.z,o.w])
        tmat=tf.transformations.quaternion_matrix(quat)
        tmat[0][3]= table.pose.pose.position.x
        tmat[1][3]= table.pose.pose.position.y
        tmat[2][3]= table.pose.pose.position.z
        
        o = viewpoint.orientation
        quat=numpy.array([o.x,o.y,o.z,o.w])
        vmat=tf.transformations.quaternion_matrix(quat)
        vmat[0][3]= viewpoint.position.x
        vmat[1][3]= viewpoint.position.y
        vmat[2][3]= viewpoint.position.z
        #mat = numpy.linalg.inv(mat)
        
        new=numpy.dot(tmat,vmat)
        
        viewpoint.position.x=new[0][3]
        viewpoint.position.y=new[1][3]
        viewpoint.position.z=new[2][3]
        viewpoint.orientation =  mat_to_quat(new)

        # create a non-interactive control which contains the box
        box_control = InteractiveMarkerControl()
        box_control.always_visible = True
        box_control.markers.append( box_marker )
        marker.controls.append( box_control )

        def add_control(orientation,name, move_rotate=1):
            control = InteractiveMarkerControl()
            control.orientation = orientation
            control.name = name
            #control.scale.x = 0.3
            #control.scale.y = 0.3
            #control.scale.z = 0.3
            if move_rotate==1:
                control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
            else:
                control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
            marker.controls.append(control)
            
        add_control(Quaternion(1,0,0,1),"move_x",1)
        add_control(Quaternion(0,1,0,1),"move_y",1)
        add_control(Quaternion(0,0,1,1),"move_z",1)
        add_control(Quaternion(1,0,0,1),"rotate_x",2)
        add_control(Quaternion(0,1,0,1),"rotate_y",2)
        add_control(Quaternion(0,0,1,1),"rotate_z",2)

        self._marker_server.insert(marker, self._viewpoint_marker_feedback)
        self._marker_server.applyChanges()

        self._marker_server.setPose( name, viewpoint )
        self._marker_server.applyChanges()


    def _viewpoint_marker_feedback(self, feedback):
        print feedback.pose
        if hasattr(self, "vp_timer_"+feedback.marker_name):
            getattr(self, "vp_timer_"+feedback.marker_name).cancel()        
        setattr(self, "vp_timer_"+feedback.marker_name,
                Timer(3, self.update_viewpoint_db, [feedback]))
        getattr(self, "vp_timer_"+feedback.marker_name).start()        

    def update_viewpoint_db(self, feedback):
        print "Updating viewpoint ", feedback.marker_name
        
        # get the table
        for table, meta in self._tables: # uggly review hack code.
            if meta["name"] == feedback.marker_name[:-10]:
                # found it
                rospy.loginfo("%s for %s"%(feedback.marker_name, meta['name']))
                break
        else:
            rospy.logerr("No table for view point %s"%feedback.marker_name)
            sys.exit(1)
                
        # Move pose into /world.
        o = table.pose.pose.orientation
        quat=numpy.array([o.x,o.y,o.z,o.w])
        tmat=tf.transformations.quaternion_matrix(quat)
        tmat[0][3]= table.pose.pose.position.x
        tmat[1][3]= table.pose.pose.position.y
        tmat[2][3]= table.pose.pose.position.z
        tmat = numpy.linalg.inv(tmat)
        
        o = feedback.pose.orientation
        quat=numpy.array([o.x,o.y,o.z,o.w])
        vmat=tf.transformations.quaternion_matrix(quat)
        vmat[0][3]= feedback.pose.position.x
        vmat[1][3]= feedback.pose.position.y
        vmat[2][3]= feedback.pose.position.z
        
        new=numpy.dot(tmat,vmat)
        
        feedback.pose.position.x=new[0][3]
        feedback.pose.position.y=new[1][3]
        feedback.pose.position.z=new[2][3]
        feedback.pose.orientation =  mat_to_quat(new)        
        
        print self._msg_store.update_named(feedback.marker_name,
                               feedback.pose);

    
    def _marker_feedback(self, feedback):
        table = [t for t, tm in self._tables if t.table_id==feedback.marker_name]
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
