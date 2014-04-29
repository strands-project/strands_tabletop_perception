#! /usr/bin/env python

import roslib; roslib.load_manifest('perceive_tabletop_action')
import rospy
import actionlib
import sys
from perceive_tabletop_action.msg import *

if __name__ == '__main__':
    rospy.init_node('perceive_tabletop_client')
    client = actionlib.SimpleActionClient('perceive_tabletop', PerceiveTabletopAction)
    client.wait_for_server()

    if len(sys.argv)!=2:
        print ("Need table name as arg")
        sys.exit(1)
        
    goal = PerceiveTabletopGoal()
    # Fill in the goal here
    goal.table_id = sys.argv[1] #"test_table_1"
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration.from_sec(5.0))
    print client.get_result()

    
