#! /usr/bin/env python
import rospy
import smach
import smach_ros
import json
import math
import sys
import copy
import random

from geometry_msgs.msg import Polygon
from geometry_msgs.msg import Point32
from geometry_msgs.msg import PoseWithCovarianceStamped

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
                             input_keys=['table_area'],
                             output_keys=['pose_output','view_list', 'action_completed', 'current_view', 'num_of_views'])



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


        # get current robot pose
        self.got_current_pose = False            

        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.amcl_cb)

        # visualizing nav goals in RVIZ
        self.pubmarker = rospy.Publisher('supporting_planes_poses', MarkerArray)
        self.marker_len = 0

        self.agenda = []
        self.current_pose_idx = 0

        self.first_call = True
        

    def amcl_cb(self, data):

        if self.got_current_pose == False:
            self.current_pose = data.pose.pose
            rospy.loginfo("Got current robot pose: (%f,%f)" % (self.current_pose.position.x, self.current_pose.position.y))
            self.got_current_pose = True

     

    def execute(self, userdata):
        rospy.loginfo('Executing state %s', self.__class__.__name__)

        userdata.current_view = self.current_pose_idx
        userdata.num_of_views = len(self.agenda)
        
        if self.first_call == False:

            
            if self.current_pose_idx < len(self.agenda):
                userdata.pose_output = self.agenda[self.current_pose_idx]
                userdata.view_list = [[0.0,0.5]] # ,[0.5,0.5],[-0.5,0.5]]
                userdata.action_completed = False
                self.current_pose_idx += 1
                return 'succeeded'
            
            elif self.current_pose_idx == len(self.agenda):
                userdata.action_completed = True
                
                rospy.sleep(4) # sleep to see update in feedback
                return 'action_completed'


        # otherwise re-sample new goals
        try:
            self.first_call = False
            
            self.num_of_nav_goals =   int(rospy.get_param('num_of_nav_goals', '100'))
            self.num_of_trajectories = int(rospy.get_param('num_of_trajectories', '500'))
            
            min_radius = float(rospy.get_param('min_radius', '0.40'))
            max_radius = float(rospy.get_param('max_radius', '2.0'))

            coverage_total = float(rospy.get_param('coverage_total', '0.8'))
            coverage_avg = float(rospy.get_param('coverage_avg', '2.0'))
            
            # TODO: compute area in proximity to table given table_pose and table_area
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
            min_max_polygon = [[min_x - max_radius , min_y - max_radius ],
                               [min_x - max_radius , max_y + max_radius ],
                               [max_x + max_radius , max_y + max_radius ],
                               [max_x + max_radius , min_y - max_radius ]]
            points = []
            for point in min_max_polygon:
                rospy.loginfo('Point: %s', point)
                points.append(Point32(float(point[0]),float(point[1]),0))

            poly = Polygon(points)
            

            # TODO: sample once, order views, use views on agenda (only re-sample if necessary)
            nav_goals_resp = self.nav_goals(self.num_of_nav_goals, min_radius, poly)

            nav_goals_eval_resp = self.nav_goals_eval(nav_goals_resp.goals, polygon, coverage_total, coverage_avg)

            viewpoints = create_viewpoints(nav_goals_eval_resp.sorted_goals.poses,
                                           nav_goals_eval_resp.weights,
                                           nav_goals_eval_resp.coverage_idx)
            
            while not self.got_current_pose:
                rospy.info("Waiting for current pose from amcl")
            
            vp_trajectory = plan_views(self.current_pose, viewpoints, self.num_of_trajectories, len(viewpoints))
            
            vp_trajectory_weights = get_weights(vp_trajectory)

            
            self.delete_markers()            
            markerArray = MarkerArray()


            for i in range(0,len(vp_trajectory)):
                
                self.agenda.append(vp_trajectory[i].get_pose())
                self.create_marker(markerArray,
                                   i,
                                   vp_trajectory[i].get_pose(),
                                   vp_trajectory_weights)


            rospy.loginfo("Coverage - index: %i, total: %f, avg: %f",nav_goals_eval_resp.coverage_idx,
                     nav_goals_eval_resp.coverage_total, nav_goals_eval_resp.coverage_avg)
            
            rospy.loginfo("len(traj): %i, len(agenda): %i ", len(vp_trajectory), len(self.agenda))

            self.marker_len =  len(markerArray.markers)
            self.pubmarker.publish(markerArray)
                
            self.current_pose_idx = 0
            
            rospy.loginfo("Next pose: (%s,%s)", nav_goals_eval_resp.sorted_goals.poses[self.current_pose_idx].position.x,nav_goals_eval_resp.sorted_goals.poses[self.current_pose_idx].position.y )
            
            userdata.pose_output = self.agenda[self.current_pose_idx]
            # TODO: sample PTU poses
            userdata.view_list = [[0.0,0.5]] # ,[0.5,0.5],[-0.5,0.5]]
                                
            self.current_pose_idx += 1
            
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
        marker1.color.a = 0.25

        max_idx = weights.index(max(weights))
        min_idx = weights.index(min(weights))
        
        marker1.color.r = r_func((weights[marker_id] - weights[min_idx]) / (weights[max_idx]- weights[min_idx] +1))
        marker1.color.g = g_func((weights[marker_id] - weights[min_idx]) / (weights[max_idx]- weights[min_idx] +1))
        marker1.color.b = b_func((weights[marker_id] - weights[min_idx]) / (weights[max_idx]- weights[min_idx] +1))

        #rospy.loginfo("weight: %s max: %s ratio: %s",weights[marker_id], weights[0], weights[marker_id] / weights[0])
        #rospy.loginfo("ID: %s RGB: %s %s %s", marker_id, marker1.color.r, marker1.color.g, marker1.color.b)

        marker1.pose.orientation = pose.orientation
        marker1.pose.position = pose.position
        marker1.points = [Point(0,0,0.01),Point(2,-0.75,0.01),Point(2,0.75,0.01)]
        
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



class Viewpoint():

    def __init__(self, pose, weight, prob):

        self.pose = pose
        self.weight = weight
        self.prob = prob

    def get_pose(self):

        return self.pose
    
    def get_weight(self):

        return self.weight

    def get_prob(self):

        return self.prob
    

def plan_views(current_pose, viewpoints, traj_num=100, traj_len=10):

    costs = calc_costs(viewpoints)

    traj = sample_trajectories(viewpoints, traj_num, traj_len)
    
    traj_costs = evalaute_trajectories(current_pose, traj, costs)

    min_traj = get_min_traj(traj_costs, traj)

    return min_traj 

def get_weights(viewpoints):

    weights = []
    
    for vp in viewpoints:
        weights.append(vp.get_weight())
        
    return weights

def create_viewpoints(pose, weight, idx):

    viewpoints = []

    wsum = sum(weight)
    
    for i in range(0,idx):
        vp = Viewpoint(pose[i],weight[i],float(weight[i])/float(wsum))
        viewpoints.append(vp)
        
    return viewpoints


def calc_costs(viewpoints):

    costs = dict() 

    for i in range(len(viewpoints)):

        costs2 = dict()

        for j in range(len(viewpoints)):

            if i != j:
                p1 = viewpoints[i].get_pose()
                p2 = viewpoints[j].get_pose()

                dist = math.sqrt( (p1.position.x - p2.position.x)**2 +  (p1.position.y - p2.position.y)**2 )   
                
                costs2[viewpoints[j]] = dist

        costs[viewpoints[i]] = costs2
    
    return costs

def sample_trajectories(viewpoints, number, length):

    pop0 = []
    for i in range(len(viewpoints)):
        #if i < length:
        pop0.extend([i for x in range(int(viewpoints[i].get_weight()) - 1 ) ])
        #else:
         #   break

    if len(pop0) < length:
        return []
    
    traj_lst = []
        
    for i in range(number):

        popn = copy.deepcopy(pop0)
        
        traj = []
        for j in range(length):

            if len(popn) < 1:
                return []
                
            idx = random.sample(popn,1)
            popn = [x for x in popn if x != idx[0]]

            traj.append(viewpoints[idx[0]])

        traj_lst.append(traj)
            
    return traj_lst

def evalaute_trajectories(current_pose, trajectories, costs):

    traj_costs = []
    
    for traj in trajectories:
        c = 0

        p1 = current_pose 
        p2 = traj[0].get_pose()

        dist = math.sqrt( (p1.position.x - p2.position.x)**2 +  (p1.position.y - p2.position.y)**2 )   
        c += dist

        prob = 1
        
        for i in range(len(traj)-1):

            prob -= traj[i].get_prob()

            c += costs[traj[i]][traj[i+1]] * prob
            
        traj_costs.append(c)

    return traj_costs


def get_min_traj(traj_costs, trajectories):

    if len(trajectories) == 0:
        return []
    
    min_idx = traj_costs.index(min(traj_costs))

    return trajectories[min_idx]



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


    
