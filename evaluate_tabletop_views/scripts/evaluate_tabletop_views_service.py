#!/usr/bin/env python
import roslib; roslib.load_manifest('nav_goals_generator')
import rospy
import tf

from tf.transformations import euler_from_quaternion, quaternion_from_euler


from nav_goals_generator.srv import *
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point32
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid

from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

import random
from math import sin, cos
import numpy
import sys

################################################################
# Ray-casting algorithm
# 
# adapted from http://rosettacode.org/wiki/Ray-casting_algorithm
################################################################
_eps = 0.00001

def ray_intersect_seg(p, a, b):
    ''' takes a point p and an edge of two endpoints a,b of a line segment returns boolean
    '''
    if a.y > b.y:
        a,b = b,a
    if p.y == a.y or p.y == b.y:
        p = Point32(p.x, p.y + _eps, 0)
 
    intersect = False
 
    if (p.y > b.y or p.y < a.y) or (
        p.x > max(a.x, b.x)):
        return False
 
    if p.x < min(a.x, b.x):
        intersect = True
    else:
        if abs(a.x - b.x) > sys.float_info.min:
            m_red = (b.y - a.y) / float(b.x - a.x)
        else:
            m_red = sys.float_info.max
        if abs(a.x - p.x) > sys.float_info.min:
            m_blue = (p.y - a.y) / float(p.x - a.x)
        else:
            m_blue = sys.float_info.max
        intersect = m_blue >= m_red
    return intersect
 
def is_odd(x): return x%2 == 1
 
def is_inside(p, poly):
    ln = len(poly)
    num_of_intersections = 0
    for i in range(0,ln):
        num_of_intersections += ray_intersect_seg(p, poly[i], poly[(i + 1) % ln])

    return is_odd(num_of_intersections)

def create_viewcone(pose, viewcone_length, viewcone_width):

    (roll,pitch,theta) = euler_from_quaternion([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
  
    ox = pose.position.x
    oy = pose.position.y

    aax = ox + viewcone_length
    aay = oy - viewcone_width/2.0

    bbx = ox + viewcone_length
    bby = oy + viewcone_width/2.0
  
    ax = cos(theta) * (aax-ox) - sin(theta) * (aay-oy) + ox
    ay = sin(theta) * (aax-ox) + cos(theta) * (aay-oy) + oy 
  
    bx = cos(theta) * (bbx-ox) - sin(theta) * (bby-oy) + ox
    by = sin(theta) * (bbx-ox) + cos(theta) * (bby-oy) + oy

    return [ Point32(ox,oy,0),  Point32(ax,ay,0),  Point32(bx,by,0)] 
    
################################################################
# Viewpoint Evaluation
################################################################

class ViewpointEvaluation():
	"A class for the evaluation of tabletop views"

	def __init__(self):
            rospy.init_node('nav_goals_evaluation_service')
            rospy.loginfo("Started nav_goals_evaluation service")


            # subscribing to a map
            self.map_frame = rospy.get_param('~map_frame', '/map')
            rospy.loginfo("Evaluating goals in %s", self.map_frame)

            self.viewcone_length = rospy.get_param('viewcone_length', '3.0')
            self.viewcone_width  = rospy.get_param('viewcone_width', '1.0')

            rospy.loginfo("Evaluating viewcones of length: %f, width: %f", self.viewcone_length, self.viewcone_width)
            
            # setting up the service
            self.ser = rospy.Service('/nav_goals_evaluation', ROINavGoals, self.evaluate)

            rospy.spin()
            rospy.loginfo("Stopped nav_goals_evaluation service")

        def map_callback(self,data):

            # get map data
            self.resolution = data.info.resolution
            self.width = data.info.width
            self.height = data.info.height
            self.origin = data.info.origin
            self.data = data.data

            self.map_min_x = self.origin.position.x
            self.map_max_x = self.origin.position.x + self.width * self.resolution
            self.map_min_y = self.origin.position.y
            self.map_max_y = self.origin.position.y + self.height * self.resolution


        def evaluate(self,req):
            rospy.loginfo('Incoming service request')

            self.roi = req.roi
            
            res = ROINavGoalsResponse()

            try:
                msg = rospy.wait_for_message(self.map_frame, OccupancyGrid , timeout=10.0)
                self.map_callback(msg)
            except rospy.ROSException, e:
                rospy.logwarn("Failed to get %s" % self.map_frame)
                return res
            
            # process arguments
            self.process_arguments()

            # generate response
            res.sorted_goals.header.frame_id = '/map' # self.map_frame

            goals = req.goals.poses
            weights = [1.0 for n in range(0,len(req.goals.poses))]

            keys = []
            key_idx_count = dict()
            keys_at_pose = [[] for p in goals]
            
            for cell_x in range(int(self.cell_min_x), int(self.cell_max_x)):
                for cell_y in range(int(self.cell_min_y), int(self.cell_max_y)):

                    pose = Pose()
                    pose.position.x = cell_x * self.resolution + self.origin.position.x
                    pose.position.y = cell_y * self.resolution + self.origin.position.y
                    
                    
                    # if the point lies within ROI of table 
                    if self.in_roi(pose.position.x,pose.position.y):
                        #rospy.loginfo("Point inside table polygon.")
                        # check whether point is in viewcone

                        keys.append(pose)
                        key_idx_count[len(keys)-1] = 0
                        
                        for i, goal in enumerate(req.goals.poses):
                            viewcone = create_viewcone(goal,self.viewcone_length, self.viewcone_width)
                            # test whether voxel is in viewcone
                            p = Point32(pose.position.x,pose.position.y,0)
                            if is_inside(p, viewcone):
                                weights[i] += 1.0
                                keys_at_pose[i].append(pose)
                            
                    # else:
                    #     rospy.loginfo("Point outside table polygon: ignore.")
                
            rospy.loginfo(weights)

            sorted_goals = [y for (x,y) in sorted(zip(weights,goals), reverse=True)]
            sorted_keys_at_pose = [y for (x,y) in sorted(zip(weights,keys_at_pose), reverse=True)]
            sorted_weights = sorted(weights,reverse=True)

            rospy.loginfo(sorted_weights)

            rospy.loginfo("Sorted goal poses:")
            coverage_idx = 0
            coverage_total = 0.0 
            coverage_avg = 0.0

            for i, g in enumerate(sorted_goals):

                rospy.loginfo("%i POSE (%f,%f), WEIGHT %f", i, g.position.x, g.position.y, sorted_weights[i]);

                if coverage_total < req.coverage_total or coverage_avg < req.coverage_avg:

                    for k_at_p in sorted_keys_at_pose[i]:

                        for kidx, k in enumerate(keys):
                            if k == k_at_p:
                                key_idx_count[kidx] +=1
                                continue

                    voxels_all = 0
                    voxels_covered = 0
                    voxels_viewcount = 0

                    for k_idx in key_idx_count:

                        voxels_all += 1

                        if key_idx_count[k_idx] > 0:
                            voxels_covered += 1
                            voxels_viewcount += key_idx_count[k_idx]

                    coverage_total = float(voxels_covered) /  float(voxels_all)
                    coverage_avg =   float(voxels_viewcount) / float(voxels_covered)
                    coverage_idx += 1

            res.sorted_goals.poses = sorted_goals
            res.weights = sorted_weights
                    
            res.coverage_idx = coverage_idx
            res.coverage_total = coverage_total
            res.coverage_avg = coverage_avg

            rospy.loginfo("Coverage - index: %i, total: %f, avg: %f", res.coverage_idx, res.coverage_total, res.coverage_avg)
            rospy.loginfo("Finished evaluation. Sending back response ...")
            return res

        def process_arguments(self):

            # region of interest (roi) must lie inside the map boundaries
            # if roi is empty, the whole map is treated as roi by default
            if len(self.roi.points) == 0:
                self.bbox_min_x = self.map_min_x
                self.bbox_max_x = self.map_max_x
                self.bbox_min_y = self.map_min_y
                self.bbox_max_y = self.map_max_y
                rospy.loginfo('no ROI specified, full map is used.')

            else:
                # if the roi is outside the map, adjust to map boundaries
                # determine bbox of roi
                self.bbox_min_x = float('inf')
                self.bbox_max_x = float('-inf')
                self.bbox_min_y = float('inf')
                self.bbox_max_y = float('-inf')
                for p in self.roi.points:
                    if p.x < self.map_min_x:
                        p.x = self.map_min_x
                    if p.x > self.map_max_x:
                        p.x = self.map_max_x
                        
                    if p.x < self.bbox_min_x:
                        self.bbox_min_x = p.x
                    if p.x > self.bbox_max_x:
                        self.bbox_max_x = p.x
                                                
                    if p.y < self.map_min_y:
                        p.y = self.map_min_y
                    if p.y > self.map_max_y:
                        p.y = self.map_max_y
                        
                    if p.y < self.bbox_min_y:
                        self.bbox_min_y = p.y
                    if p.y > self.bbox_max_y:
                        self.bbox_max_y = p.y
                        
            # calculate bbox for cell array
            self.cell_min_x = int((self.bbox_min_x - self.origin.position.x) / self.resolution)
            self.cell_max_x = int((self.bbox_max_x - self.origin.position.x) / self.resolution)
            self.cell_min_y = int((self.bbox_min_y - self.origin.position.y) / self.resolution)
            self.cell_max_y = int((self.bbox_max_y - self.origin.position.y) / self.resolution)

            rospy.loginfo('ROI bounding box (meters): (%s,%s) (%s,%s)', \
                              self.bbox_min_x,self.bbox_min_y,self.bbox_max_x,self.bbox_max_y)

            rospy.loginfo('ROI bounding box (cells): (%s,%s) (%s,%s)', \
                          self.cell_min_x,self.cell_min_y,self.cell_max_x,self.cell_max_y)
                

        
        def cell(self, x,y):
                if x < 0 or y <0 or x >= self.width or y >= self.height:
                    #rospy.loginfo("out of bounds! x: %s, y: %s", x, y)
                    # return 'unknown' if out of bounds
                    return -1
                
                return self.data[x +  self.width * y]

        def in_roi(self,x,y):
            if (len(self.roi.points)==0):
                return True
            p = Point32(x,y,0)
            return is_inside(p, self.roi.points)
        
        def in_collision(self,x,y):
                x_min = x - self.inflated_footprint_size
                x_max = x + self.inflated_footprint_size                    
                y_min = y - self.inflated_footprint_size
                y_max = y + self.inflated_footprint_size                    
                for i in range(x_min,x_max):
                        for j in range(y_min,y_max):
                                if (self.cell(i,j) != 0):
                                        return True
                return False
                        

if __name__ == '__main__':
    ViewpointEvaluation()
