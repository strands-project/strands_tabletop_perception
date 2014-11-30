#!/usr/bin/python

import rospy
from world_state import report

import world_state.geometry as geometry
from world_state.state import World, Object
from world_state.observation import MessageStoreObject, Observation, TransformationStore
from world_state.identification import ObjectIdentification

import math
import numpy as np


from classifier_srv_definitions.srv import segment_and_classify
from table_segmentation.srv import SegmentTable
from sensor_msgs.msg import PointCloud, PointCloud2

import web
import cv2
import os
import json

from threading import Lock

active_lock = Lock()

import qsr_kb.srv

### Templates
render = web.template.render('../resources/results_gui', base='base', globals={'report': report,})
        
urls = (
    '/test(.*)', 'Test', 
    '/tables(.*)',  'ListTables', 
    '/view_table/(.*)', 'ListTableViews',
    '/observation/(.*)/(.*)', 'Observation',
    '/get_img(.*)/(.*)/(.*)', 'GetImage',
    '/show_pointclouds/(.*)/(.*)', 'ShowPointClouds',
    '/calculate_qsr/(.*)/(.*)', 'CalculateQSRs',
    '/visualise_qsr', 'VisualiseQSRs',
    '/resegment/(.*)/(.*)', 'Resegment'
)
                          
app = web.application(urls, globals())

class Test(object):        
    def GET(self, name):
        if not name: 
            name = 'World'
        return 'Hello, ' + name + '!'
    
class ListTables(object):
    def GET(self, table_name):
        tables = report.get_tables_list()
        return render.tables_list(tables)
    
class ListTableViews(object):
    def GET(self, table_name):
        observations = report.get_table_observations(table_name)
        return render.table_view(table_name, observations)
    
class Observation(object):
    def GET(self, table_name, observation_stamp):
        #image =  report.create_table_observation_image(table_name, float(observation_stamp))
        #cv2.imsave(image, "/tmp/image.png")
        #observations = report.get_table_observations(table_name)
        objects =  report.get_objects_observed(table_name, float(observation_stamp))
        return render.observation(table_name, observation_stamp, report.epoch_to_str_time(float(observation_stamp)),
                                  objects)
    
    
class ShowPointClouds(object):
    def GET(self, table_name, observation_stamp):
        if not hasattr(self, "visualiser"):
            self.visualiser =  report.PointCloudVisualiser()
        self.visualiser.clear()
        #objects =  report.get_objects_observed(table_name, float(observation_stamp))
        clouds =  report.get_tabletop_object_clouds(table_name, float(observation_stamp))
        #for ob in objects:
        for pointcloud in clouds:
            #pointcloud = ob._point_cloud.retrieve()
            self.visualiser.add_cloud(pointcloud)
        self.visualiser.publish()
        return "ok"
    
class CalculateQSRs(object):
    def GET(self, table_name, observation_stamp):
        qsrs = report.calculate_qsrs(table_name, float(observation_stamp))
        t = ""
        for q in qsrs:
            t += "<p>" + str(q)
        return t
    

    
def make_query(query):
    rospy.wait_for_service('qsrvis')
    try:
        swipl = rospy.ServiceProxy('qsrvis', qsr_kb.srv.PrologQuery)
        resp = swipl(query)
        return json.loads(resp.solution)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

# Get all relations in the scene. 
query = "qsrT(R,O1,O2,Q), vis([Q])."

#print "Asking for relations: %s"%(query)
#print "Solution: %s"%(make_query(query))

#query = "next_vis(X)."
#sol = make_query(query)
#while sol:
    #key = raw_input("Press Enter to visualise next solution...")
    #sol = make_query(query)
    
class VisualiseQSRs(object):
    def POST(self):
        i = web.input()
        query = i.prolog
        print "Making query ", query
        return make_query(query)

    
    
class GetImage(object):
    def GET(self, typ, table_name, observation_stamp):
        with active_lock:
            print table_name, "  ", observation_stamp, "  ", typ
            if typ ==  "_raw":
                image =  report.get_table_observation_rgb(table_name, float(observation_stamp))
            else:
                image =  report.create_table_observation_image(table_name, float(observation_stamp))
            cv2.imwrite("/tmp/image.png", image)
            with open("/tmp/image.png", "rb") as f:
                data = f.read()
            web.header("Content-Type", "images/png") 
            return data
        
class Resegment(object):
    def GET(self, table_name, observation_stamp):
        # Get the services for segmentation and classificiation
        rospy.loginfo("Waiting for /classifier_service/segment_and_classify and "
                      "/table_segmentation/segment_table services.")
        try:
            rospy.wait_for_service('/classifier_service/segment_and_classify', 10)
            rospy.wait_for_service('/segment_table', 10)
        except:
            return "Error getting ROS services for segmentation"

        try:
            self.table_seg = rospy.ServiceProxy('/segment_table', SegmentTable )
            self.obj_rec = rospy.ServiceProxy('/classifier_service/segment_and_classify', segment_and_classify )
        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: %s" % e)
            pass
        
        self._pub = rospy.Publisher("pointcloud_visualise", PointCloud2)
        return self.update_table_classification(table_name, float(observation_stamp))
    
    def update_table_classification(self, table_name,  observation_stamp):
        world = World()
        table =  world.get_object(table_name)
        
        # Delete all children of the table that have observation in common

        # Which table observation is closest to timestamp
        if len(table._observations) < 1:
            raise Exception("Table has no observations")
        closest =  min([(ob, math.fabs(ob.stamp - observation_stamp)) for ob in table._observations],
                       key=lambda x: x[1])
        observation = closest[0]
        
        children = world.get_children(table_name, {'_observations': {'$elemMatch': {'stamp': observation.stamp}}})
        start = rospy.Time.now().to_time()
        end = 0
        for c in children:
            start = min(start, c._life_start)
            if c._life_end is not None:
                end = max(end, c._life_end)
            world.remove_object(c)
        if end == 0:
            end = None

        #return "Removed %d children" % len(children)
    
    
        
        pointcloud = observation.get_message('/head_xtion/depth/points')
        tf =  TransformationStore.msg_to_transformer(observation.get_message("/tf"))
        
        depth_to_base_link = tf.lookupTransform("/map", pointcloud.header.frame_id, 
                                            pointcloud.header.stamp)
        depth_to_base_link = geometry.Pose(geometry.Point(*(depth_to_base_link[0])),
                                       geometry.Quaternion(*(depth_to_base_link[1]))).as_homog_matrix()
        try:
            print depth_to_base_link
            seged = self.table_seg(pointcloud, table_name, depth_to_base_link.flatten())
            obj_rec_resp = self.obj_rec(seged.cloud, depth_to_base_link.flatten())
            seged.cloud.header.frame_id = "/map"
            #self._pub.publish(seged.cloud)
            self._pub.publish(pointcloud)
        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: %s" % e)
        
        objects = obj_rec_resp.class_results
        
        depth_to_world = tf.lookupTransform("/map", pointcloud.header.frame_id, 
                                            pointcloud.header.stamp)
        depth_to_world = geometry.Pose(geometry.Point(*(depth_to_world[0])),
                                       geometry.Quaternion(*(depth_to_world[1])))
        table_to_world = table.pose
        depth_to_table = np.dot(np.linalg.inv(table_to_world.as_homog_matrix()),
                                depth_to_world.as_homog_matrix() )
        
        if len(objects) == 0:
            rospy.loginfo("view: nothing perceived")
        else:
            rospy.loginfo('view: found objects: %i', len(objects))
        
            for j in range(len(objects)):
                new_object =  world.create_object()
                new_object._life_start = start
                new_object._life_end = end
                
                
                # The position (centroid)
                position = geometry.Point.from_ros_point32(obj_rec_resp.centroid[j])
                position.transform(depth_to_table)
                pose = geometry.Pose(position)
                new_object.add_pose(pose) # no orientation
        
                depth_to_object = np.dot(np.linalg.inv(pose.as_homog_matrix()), depth_to_table)
                
                # The point cloud
                cloud = geometry.transform_PointCloud2(obj_rec_resp.cloud[j],
                                                             depth_to_object,
                                                             "/map")
                new_object._point_cloud =  MessageStoreObject.create(cloud)
                
                # The bounding box
                bbox_array =  []
                for pt in obj_rec_resp.bbox[j].point:
                    p =  geometry.Point.from_ros_point32(pt)
                    p.transform(depth_to_object)
                    bbox_array.append([p.x, p.y, p.z])
                bbox = geometry.BBoxArray(bbox_array)
                new_object._bounding_box = bbox
        
                new_object.add_observation(observation) 
                table.add_child(new_object)
        
                # The classification
                classification = {}
                for cl, conf in zip(obj_rec_resp.class_results[j].class_type,
                                    obj_rec_resp.class_results[j].confidence):
                    classification[cl.data.strip('/')] = conf
                new_object.add_identification("SingleViewClassifier",
                                              ObjectIdentification(classification))
                


                
    
    
if __name__ == "__main__":
    print "Init ROS node."
    rospy.init_node("table_observations_gui")
    print "Web server starting.."
    app.run()