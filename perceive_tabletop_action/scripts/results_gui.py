#!/usr/bin/python

import rospy
from world_state import report

import web
import cv2
import os
import json

from threading import Lock

active_lock = Lock()

import qsr_kb.srv

### Templates
render = web.template.render('../resources/results_gui', base='base')
        
urls = (
    '/test(.*)', 'Test', 
    '/tables(.*)',  'ListTables', 
    '/view_table/(.*)', 'ListTableViews',
    '/observation/(.*)/(.*)/(.*)/(.*)/(.*)', 'Observation',
    '/get_img(.*)/(.*)/(.*)', 'GetImage',
    '/show_pointclouds/(.*)/(.*)', 'ShowPointClouds',
    '/calculate_qsr/(.*)/(.*)', 'CalculateQSRs',
    '/visualise_qsr', 'VisualiseQSRs'
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
    def GET(self, table_name, view, view_count, observation_nice_stamp, observation_stamp):
        #image =  report.create_table_observation_image(table_name, float(observation_stamp))
        #cv2.imsave(image, "/tmp/image.png")
        #observations = report.get_table_observations(table_name)
        objects =  report.get_objects_observed(table_name, float(observation_stamp))
        return render.observation(table_name, observation_stamp, observation_nice_stamp,
                                  objects, int(view)+1, view_count)
    

    
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

if __name__ == "__main__":
    print "Init ROS node."
    rospy.init_node("table_observations_gui")
    print "Web server starting.."
    app.run()