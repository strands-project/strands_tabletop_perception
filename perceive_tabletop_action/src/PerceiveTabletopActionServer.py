#! /usr/bin/env python
import roslib; roslib.load_manifest('perceive_tabletop_action')
import rospy

from smach_ros import ActionServerWrapper

from  perceive_tabletop_action.msg import *
from  perceive_tabletop.state_machine import PerceiveTabletopSM

class PerceiveTabletopActionServer:

    def __init__(self, server_name):

        self.server_name = server_name
        self.pta_sm = PerceiveTabletopSM()
        
        
    def run_server(self):

        # Construct action server wrapper
        asw = ActionServerWrapper(
            self.server_name, PerceiveTabletopAction,
            wrapped_container = self.pta_sm,
            succeeded_outcomes = ['succeeded'],
            aborted_outcomes = ['aborted'],
            preempted_outcomes = ['preempted'],
            goal_key = 'goal')

        # Run the server in a background thread
        asw.run_server()

        # Wait for control-c
        rospy.spin()
        

if __name__ == '__main__':
  rospy.init_node('perceive_tabletop_server')
  ptas = PerceiveTabletopActionServer('perceive_tabletop')
  ptas.run_server()
