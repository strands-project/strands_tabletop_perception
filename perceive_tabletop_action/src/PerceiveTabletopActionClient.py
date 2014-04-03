#! /usr/bin/env python

import roslib; roslib.load_manifest('perceive_tabletop_action')
import rospy
import actionlib

from perceive_tabletop_action.msg import *

if __name__ == '__main__':
    rospy.init_node('perceive_tabletop_client')
    client = actionlib.SimpleActionClient('perceive_tabletop', PerceiveTabletopAction)
    client.wait_for_server()

    goal = PerceiveTabletopGoal()
    # Fill in the goal here
    goal.table_id = "table15"
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration.from_sec(5.0))
    print client.get_result()

    
