#!/usr/bin/python
import rospy
from learn_objects_action.machine import LearnObjectActionMachine
from smach_ros import ActionServerWrapper
from learn_objects_action.msg import LearnObjectAction

rospy.init_node("learn_dynamic_object_action_server")

# Construct state machine
sm = LearnObjectActionMachine()

# Construct action server wrapper
asw = ActionServerWrapper(
    'learn_object',
    LearnObjectAction,
    wrapped_container = sm,
    succeeded_outcomes = ['succeded'], 
    aborted_outcomes = ['failed'],
    preempted_outcomes = ['preempted'], 
    goal_key = 'action_goal',
    result_key='action_result' )

# Run the server in a background thread
asw.run_server()

# Wait for control-c
rospy.spin()
