#! /usr/bin/env python
import rospy
import smach
import smach_ros

class ActionMonitor(smach.State):

    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['succeeded', 'aborted', 'preempted'],
                             input_keys=['goal'],
                             output_keys=['goal'],
                             )
                                                  
    def execute(self, userdata):
        return 'succeeded'
    
        # action_server_name=userdata.goal.action_server
        # action_client= actionlib.SimpleActionClient(action_server_name, MoveBaseAction)
        # action_client.wait_for_server()
        # action_client.send_goal(userdata.goal)
        # status= action_client.get_state()
        # while status==GoalStatus.PENDING or status==GoalStatus.ACTIVE:   
        #     status= action_client.get_state()
        #     if self.preempt_requested():
        #         action_client.cancel_goal()
        #         self.service_preempt()
        #     action_client.wait_for_result(rospy.Duration(0.2))
        
        # if status == GoalStatus.SUCCEEDED:
        #     userdata.n_nav_fails = 0
        #     return 'succeeded'
        # elif status==GoalStatus.PREEMPTED:
        #     return 'preempted'
        # else:
        #     userdata.n_nav_fails = userdata.n_nav_fails + 1
        #     return 'aborted'
    
