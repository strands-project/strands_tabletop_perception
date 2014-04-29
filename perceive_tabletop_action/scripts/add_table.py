#! /usr/bin/env python
import roslib; roslib.load_manifest('perceive_tabletop_action')
import rospy

import sys

from strands_perception_msgs.msg import Table
from geometry_msgs.msg import PoseWithCovariance, Polygon, Point32

from ros_datacentre.message_store import MessageStoreProxy



if __name__ == '__main__':

    rospy.init_node('add_table_node')
    
    if len(sys.argv)!=3:
        print ("Usage: add_table.py <table_id> <polygon>")
        sys.exit(1)

    my_table = Table()
    my_table.table_id = sys.argv[1] # "table27"
    my_table.header.frame_id = "/map"  # The parent frame that the table is in
    
    table_pose = PoseWithCovariance()  # The transformation to the table frame
    # Fill in the table position...
    my_table.pose = table_pose

    pstr = sys.argv[2].split(',')

    lst = []
    for i in range(len(pstr)/2):
        p = []
        p.append(float(pstr[2*i]))
        p.append(float(pstr[2*i+1]))
        lst.append(p)
        
    polygon = lst  #[[3.11,2.12], [5.09,2.12], [5.09,3.3], [3.11, 3.3]]
    
    rospy.loginfo('Polygon: %s', polygon)
    points = []
    for point in polygon:
        points.append(Point32(float(point[0]),float(point[1]),0))

    poly = Polygon(points) 

    # Fill in the points in the polygon....
    my_table.tabletop = poly
    
    _msg_store = MessageStoreProxy()
    # Store the table
    _msg_store.insert(my_table)
