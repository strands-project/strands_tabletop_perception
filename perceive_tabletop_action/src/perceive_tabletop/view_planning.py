#! /usr/bin/env python
import rospy
import smach
import smach_ros
import json
import math
import sys

from geometry_msgs.msg import Polygon
from geometry_msgs.msg import Point32

from nav_goals_msgs.srv import NavGoals
from nav_goals_msgs.srv import ROINavGoals

from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

class ViewPlanning(smach.State):

    """
    Select next search pose

    """
    
    def __init__(self):

        smach.State.__init__(self,
                             outcomes=['succeeded', 'aborted', 'preempted', 'action_completed'],
                             input_keys=['table_pose','table_area'],
                             output_keys=['pose_output','view_list', 'action_completed'])



        rospy.wait_for_service('nav_goals')
        try:
            self.nav_goals = rospy.ServiceProxy('nav_goals', NavGoals)
        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: %s" % e)

        rospy.wait_for_service('nav_goals_evaluation')
        try:
            self.nav_goals_eval = rospy.ServiceProxy('nav_goals_evaluation', ROINavGoals)
        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: %s" % e)

        # visualizing nav goals in RVIZ
        self.pubmarker = rospy.Publisher('supporting_planes_poses', MarkerArray)
        self.marker_len = 0

        self.agenda = []
        self.current_pose = 0
     

    def execute(self, userdata):
        rospy.loginfo('Executing state %s', self.__class__.__name__)

        if self.current_pose < len(self.agenda):
            userdata.pose_output = self.agenda[self.current_pose]
            userdata.view_list = [[0.0,0.5]] # ,[0.5,0.5],[-0.5,0.5]]
            userdata.action_completed = False
            self.current_pose += 1
            if self.current_pose == len(self.agenda):
                userdata.action_completed = True
                return 'action_completed'
            return 'succeeded'

        # otherwise re-sample new goals
        try:

            num_of_nav_goals =   int(rospy.get_param('num_of_nav_goals', '100'))
            inf_radius       = float(rospy.get_param('inflation_radius', '0.7'))
            inf_radius_coeff = float(rospy.get_param('inflation_radius_coeff', '3.5'))

            coverage_total = float(rospy.get_param('coverage_total', '0.8'))
            coverage_avg = float(rospy.get_param('coverage_avg', '2.0'))
            
            # TODO: compute area in proximity to table given table_pose and table_area
            table_pose = userdata.table_pose
            polygon    = userdata.table_area

            min_x = float(sys.maxint)
            min_y = float(sys.maxint)
            max_x = float(-sys.maxint-1)
            max_y = float(-sys.maxint-1)
            

            # get 2D bounding box
            for point in polygon.points:
                if point.x < min_x:
                    min_x = point.x
                elif point.x > max_x:
                    max_x = point.x
                if point.y < min_y:
                    min_y = point.y
                elif point.y > max_y:
                    max_y = point.y

            # define area around table
            min_max_polygon = [[min_x - inf_radius_coeff * inf_radius, min_y - inf_radius_coeff * inf_radius],
                               [min_x - inf_radius_coeff * inf_radius, max_y + inf_radius_coeff * inf_radius],
                               [max_x + inf_radius_coeff * inf_radius, max_y + inf_radius_coeff * inf_radius],
                               [max_x + inf_radius_coeff * inf_radius, min_y - inf_radius_coeff * inf_radius]]
            points = []
            for point in min_max_polygon:
                rospy.loginfo('Point: %s', point)
                points.append(Point32(float(point[0]),float(point[1]),0))

            poly = Polygon(points)
            

            # TODO: sample once, order views, use views on agenda (only re-sample if necessary)
            nav_goals_resp = self.nav_goals(num_of_nav_goals, inf_radius, poly)

            nav_goals_eval_resp = self.nav_goals_eval(nav_goals_resp.goals, polygon, coverage_total, coverage_avg)

            self.delete_markers()            
            markerArray = MarkerArray()
            
            for i in range(0,len(nav_goals_eval_resp.sorted_goals.poses)):
                self.create_marker(markerArray,
                                   i,
                                   nav_goals_eval_resp.sorted_goals.poses[i],
                                   nav_goals_eval_resp.weights)


            self.marker_len =  len(markerArray.markers)
            self.pubmarker.publish(markerArray)
                
            self.current_pose = 0
            for i in range(0, nav_goals_eval_resp.coverage_idx):
                self.agenda.append(nav_goals_eval_resp.sorted_goals.poses[i])


            rospy.loginfo("Next pose: (%s,%s)", nav_goals_eval_resp.sorted_goals.poses[self.current_pose].position.x,nav_goals_eval_resp.sorted_goals.poses[self.current_pose].position.y )
            
            userdata.pose_output = self.agenda[self.current_pose]
            # TODO: sample PTU poses
            userdata.view_list = [[0.0,0.5]] # ,[0.5,0.5],[-0.5,0.5]]

            
        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: %s" % e)
            return 'aborted'
               
        return 'succeeded'


    def create_marker(self,markerArray, marker_id, pose, weights):
        marker1 = Marker()
        marker1.id = marker_id 
        marker1.header.frame_id = "/map"
        marker1.type = marker1.TRIANGLE_LIST
        marker1.action = marker1.ADD
        marker1.scale.x = 1
        marker1.scale.y = 1
        marker1.scale.z = 2
        marker1.color.a =   max(0.1, min(0.4, (( r_func(weights[marker_id] / weights[0]) ) / ( r_func(weights[marker_id] / weights[0]) + g_func(weights[marker_id] / weights[0]) + b_func(weights[marker_id] / weights[0]) ))))
        marker1.color.r = r_func(weights[marker_id] / weights[0])
        marker1.color.g = g_func(weights[marker_id] / weights[0])
        marker1.color.b = b_func(weights[marker_id] / weights[0])

        #rospy.loginfo("weight: %s max: %s ratio: %s",weights[marker_id], weights[0], weights[marker_id] / weights[0])
        #rospy.loginfo("ID: %s RGB: %s %s %s", marker_id, marker1.color.r, marker1.color.g, marker1.color.b)

        marker1.pose.orientation = pose.orientation
        marker1.pose.position = pose.position
        marker1.points = [Point(0,0,0.01),Point(3,-1,0.01),Point(3,1,0.01)]
        
        markerArray.markers.append(marker1)


    def delete_markers(self):
        markerArray = MarkerArray()
        for i in range(0,self.marker_len):
            marker = Marker()
            marker.header.frame_id = "/map"
            marker.id = i
            marker.action = marker.DELETE
            markerArray.markers.append(marker)
        self.pubmarker.publish(markerArray)


def trapezoidal_shaped_func(a, b, c, d, x):
  min_val = min(min((x - a)/(b - a), float(1.0)), (d - x)/(d - c))
  return max(min_val, float(0.0))


def r_func(x):
    a = -0.125
    b =  0.125
    c =  0.375
    d =  0.625

    x = 1.0 - x
  
    value = trapezoidal_shaped_func(a,b,c,d,x)
    return value

def g_func(x):
    a =  0.125
    b =  0.375
    c =  0.625
    d =  0.875
    
    x = 1.0 - x
    
    value = trapezoidal_shaped_func(a,b,c,d,x)
    return value


def b_func(x):
    a =  0.375
    b =  0.625
    c =  0.875
    d =  1.125
  
    x = 1.0 - x
  
    value = trapezoidal_shaped_func(a,b,c,d,x)
    return value


    
