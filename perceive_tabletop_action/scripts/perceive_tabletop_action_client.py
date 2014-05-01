#! /usr/bin/env python

import roslib; roslib.load_manifest('perceive_tabletop_action')
import rospy
import actionlib
import sys
from perceive_tabletop_action.msg import *


def my_feedback_cb(data):
    rospy.loginfo(data)

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
    client.send_goal(goal, feedback_cb = my_feedback_cb)
    client.wait_for_result()
    print client.get_result()

    
