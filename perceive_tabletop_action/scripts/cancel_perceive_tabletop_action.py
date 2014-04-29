#! /usr/bin/env python

import roslib; roslib.load_manifest('perceive_tabletop_action')
import rospy

import actionlib
import sys

from perceive_tabletop_action.msg import *


def cancel_client():
    # Creates the SimpleActionClient, passing the type of the action
    # to the constructor.
    client = actionlib.SimpleActionClient('perceive_tabletop', perceive_tabletop_action.msg.PerceiveTabletopAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    # Sends the goal to the action server.
    client.cancel_all_goals()


if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('perceive_tabletop_action_cancel')
        cancel_client()
    except rospy.ROSInterruptException:
        print "program interrupted before completion"
