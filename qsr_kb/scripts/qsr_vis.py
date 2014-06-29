#!/usr/bin/env python
import roslib; roslib.load_manifest('qsr_kb')

import sys
import rospy
import json
import rospkg
import re

from qsr_kb.srv import PrologQuery
from qsr_kb.srv import PrologQueryResponse
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point

from swiplclient import PrologDB
from urllib2 import HTTPError




class QSRVis(object):

    def __init__(self):
        rospy.init_node('qsrvis_server')

        PL_HOST = rospy.get_param('~PL_HOST', 'localhost')
        PL_PORT = rospy.get_param('~PL_PORT', 5000)
        PL_PASSWORD = rospy.get_param('~PL_PASSWORD', 'xyzzy') 

        # Connect to the Prolog HTTP server
        self.kb = PrologDB(PL_HOST, PL_PORT, PL_PASSWORD)

        self.s = rospy.Service('qsrvis', PrologQuery, self.handle_query)
        rospy.loginfo("QSRVis up and running")

        
        # get an instance of RosPack with the default search paths
        rospack = rospkg.RosPack()

        # get the file path for rospy_tutorials
        qsr_pl = rospack.get_path('qsr_kb') + "/src/qsr.pl"
        consult_qsr_pl = "consult('" + qsr_pl + "')"        
        self.kb.query(consult_qsr_pl)
        rospy.loginfo("QSR predicates loaded")

        # visualizing nav goals in RVIZ
        self.obj_marker = rospy.Publisher('obj_marker_array', MarkerArray)
        self.rel_marker = rospy.Publisher('rel_marker_array', MarkerArray)
        self.obj_marker_len = 0
        self.rel_marker_len = 0

        self.pattern = re.compile('next_vis\((.*)\)')
        self.create_pattern = re.compile('.*create_event.*')
               
        rospy.spin()

    def prologPose_to_ROSPose(self, ppose):
        pose = Pose()
        pose.position.x = ppose[0][0]
        pose.position.y = ppose[0][1]
        pose.position.z = ppose[0][2]
        pose.orientation.w = ppose[1][0]
        pose.orientation.x = ppose[1][1]
        pose.orientation.y = ppose[1][2]
        pose.orientation.z = ppose[1][3]
        return pose

    def handle_query(self, req):
        rospy.loginfo("Query:  %s " % (req.query))
        try:
            # create markers
            self.delete_rel_markers()  
            obj_markerArray = MarkerArray()
            rel_markerArray = MarkerArray()

            # run user query (generates vis_qsr predicates)
            json_sol = self.kb.query(req.query)
            solution = json.dumps(json_sol)
            rospy.loginfo("Solution:  %s " % (solution))

            # Visualize object and probabilities
            res = self.create_pattern.match(req.query)
            if res != None:
                self.delete_obj_markers()  
                rospy.loginfo("Visualize objects")
                json_sol1 = self.kb.query("findall_objs(Objs).")
                
                obj_lst = json_sol1[0]['Objs'] 
                
                objs = dict()
                i = 0
                for obj in obj_lst:
                    if obj[0] not in objs:
                        label = obj[2] + ' (' + str(obj[3]) + ')' 
                        self.create_obj_marker(obj_markerArray, i, self.prologPose_to_ROSPose(obj[1]), label )
                        objs[obj[0]] = obj[2]
                        i += 2

                self.obj_marker_len =  len(obj_markerArray.markers)
                self.obj_marker.publish(obj_markerArray)
                return PrologQueryResponse(solution)

            # Else: check whether query is next_vis()
            res = self.pattern.match(req.query)
            if res == None:
                # no visualization
                return PrologQueryResponse(solution)

            else:
                # visulaize relations
                qsr_var = res.group(1)
            
                if not json_sol:
                
                    return PrologQueryResponse(solution)

                qsr_lst = json_sol[0][qsr_var]

                #objs = dict()
                i = 0
                j = 0
                for qsr in qsr_lst:
                    # if qsr[1][0] not in objs:
                    #     label = qsr[1][1] if qsr[1][2] == 'None' else  qsr[1][1] + ' (' + qsr[1][2] + ')' 
                    #     self.create_obj_marker(obj_markerArray, i, self.prologPose_to_ROSPose(qsr[1][3]), label )
                    #     objs[qsr[1][0]] = qsr[1][0]
                    #     i += 2
                    # if qsr[2][0] not in objs:
                    #     label = qsr[2][1] if qsr[2][2] == 'None' else  qsr[2][1] + ' (' + qsr[2][2] + ')'
                    #     self.create_obj_marker(obj_markerArray, i, self.prologPose_to_ROSPose(qsr[2][3]), label)
                    #     objs[qsr[2][0]] = qsr[2][0]
                    #     i += 2
                    self.create_rel_marker(rel_markerArray, j, self.prologPose_to_ROSPose(qsr[1][3]), self.prologPose_to_ROSPose(qsr[2][3]), qsr[0])
                    j +=2
                
               
                self.rel_marker_len =  len(rel_markerArray.markers)
                self.rel_marker.publish(rel_markerArray)

            
        except HTTPError, e:
            rospy.logerr(e)
            return PrologQueryResponse() 
        return PrologQueryResponse(solution)

    def create_obj_marker(self, obj_markerArray, marker_id, pose, label):
        marker1 = Marker()
        marker1.id = marker_id 
        marker1.header.frame_id = "/map"
        marker1.type = marker1.SPHERE
        marker1.action = marker1.ADD
        marker1.scale.x = 0.05
        marker1.scale.y = 0.05
        marker1.scale.z = 0.05
        marker1.color.a = 1.0
        marker1.color.r = 1.0
        marker1.color.g = 1.0
        marker1.color.b = 0.0
        marker1.pose.orientation = pose.orientation
        marker1.pose.position = pose.position
        
        obj_markerArray.markers.append(marker1)

        marker2 = Marker()
        marker2.id = marker_id + 1
        marker2.header.frame_id = "/map"
        marker2.type = marker2.TEXT_VIEW_FACING
        marker2.action = marker2.ADD

        marker2.scale.z = 0.1

        marker2.color.a = 1.0
        marker2.color.r = 1.0
        marker2.color.g = 0.0
        marker2.color.b = 1.0
        marker2.pose.orientation = pose.orientation
        #marker2.pose.position = pose.position
        #marker2.pose.position.z = pose.position.z + 1.0

        x = pose.position.x 
        y = pose.position.y 
        z = pose.position.z + 0.10
        
        marker2.pose.position.x = x
        marker2.pose.position.y = y
        marker2.pose.position.z = z
        
        marker2.text = label

        obj_markerArray.markers.append(marker2)

    def create_rel_marker(self, rel_markerArray, marker_id, pose, pose2, label):
        marker1 = Marker()
        marker1.id = marker_id 
        marker1.header.frame_id = "/map"
        marker1.type = marker1.ARROW
        marker1.action = marker1.ADD
        marker1.scale.x = 0.01
        marker1.scale.y = 0.05
        marker1.scale.z = 0.15
        marker1.color.a = 1.0
        marker1.color.r = 0.0
        marker1.color.g = 0.0
        marker1.color.b = 1.0
#        marker1.pose.orientation = pose.orientation
        marker1.pose.position = pose.position
        x = pose2.position.x - pose.position.x 
        y = pose2.position.y - pose.position.y 
        z = pose2.position.z - pose.position.z 
        marker1.points = [Point(0,0,0) , Point(x,y,z)]

        rel_markerArray.markers.append(marker1)
        
        marker2 = Marker()
        marker2.id = marker_id + 1
        marker2.header.frame_id = "/map"
        marker2.type = marker2.TEXT_VIEW_FACING
        marker2.action = marker2.ADD

        marker2.scale.z = 0.20

        marker2.color.a = 1.0
        marker2.color.r = 0.0
        marker2.color.g = 1.0
        marker2.color.b = 1.0
        marker2.pose.orientation = pose.orientation

        x = pose.position.x + (pose2.position.x - pose.position.x) / 2 
        y = pose.position.y + (pose2.position.y - pose.position.y) / 2  
        z = pose.position.z + (pose2.position.z - pose.position.z) / 2  + 0.2 
        
        marker2.pose.position.x = x
        marker2.pose.position.y = y
        marker2.pose.position.z = z

        marker2.text = label

        rel_markerArray.markers.append(marker2)


    def delete_obj_markers(self):
        obj_markerArray = MarkerArray()
        for i in range(0,self.obj_marker_len):
            marker = Marker()
            marker.header.frame_id = "/map"
            marker.id = i
            marker.action = marker.DELETE
            obj_markerArray.markers.append(marker)
        self.obj_marker.publish(obj_markerArray)

    def delete_rel_markers(self):
        rel_markerArray = MarkerArray()
        for i in range(0,self.rel_marker_len):
            marker = Marker()
            marker.header.frame_id = "/map"
            marker.id = i
            marker.action = marker.DELETE
            rel_markerArray.markers.append(marker)
        self.rel_marker.publish(rel_markerArray)



if __name__ == "__main__":
    QSRVis()

# def send_query(query):
#     rospy.wait_for_service('kb')
#     try:
#         kb_server = rospy.ServiceProxy('kb', PrologQuery)
#         solution = kb_server(query)
#         return solution
#     except rospy.ServiceException, e:
#         rospy.logerr("Service call failed: %s",e)

# def usage():
#     return "%s [x y]"%sys.argv[0]



# if __name__ == "__main__":

#     rospy.init_node('kb_client')
    
#     if len(sys.argv) == 2:
#         query = str(sys.argv[1])
#     else:
#         rospy.loginfo(usage())
#         sys.exit(1)
#     rospy.loginfo("Query: %s", query)
#     soln = send_query(query)
#     rospy.loginfo("Solution: %s", soln.solution)
