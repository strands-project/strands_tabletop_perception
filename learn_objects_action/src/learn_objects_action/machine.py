# The overall action machine for the action
import rospy
import smach
import copy

import dynamic_reconfigure.client

from metric_sweep import MetricSweep,  SelectCluster
from ptu_track import ( TurnPTUToObject,  StartTransformation,
                        StopSendingTransformation, StartPTUTrack, StopPTUTrack)
from vision import StartCameraTrack,  StopCameraTrack, LearnObjectModel
from control import TravelAroundObject
from learn_objects_action.msg import LearnObjectResult

class LearnObjectActionMachine(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self,
                                    outcomes=['succeeded', 'failed', 'preempted'],
                                    input_keys=['action_goal'], 
                                    output_keys=['action_result'], 
                                    )
        #self.userdata.waypoint = waypoint
        self.userdata.action_result =  LearnObjectResult()
        
        with self:
            smach.StateMachine.add('METRIC_MAP_SWEEP', MetricSweep(),
               transitions={'done':'SELECT_CLUSTER',
                            'failed':'failed',
                            'preempted': 'preempted'})
            smach.StateMachine.add('SELECT_CLUSTER', SelectCluster(),
                transitions={'selected':'POINT_PTU',
                             'none':'failed'})
            smach.StateMachine.add('POINT_PTU', TurnPTUToObject(),
                transitions={'error':'failed',
                             'success':'START_TRANSFORM'})
            smach.StateMachine.add('START_TRANSFORM', StartTransformation(),
                transitions={'error':'failed',
                             'success':'START_PTU_TRACK'})
            smach.StateMachine.add('START_PTU_TRACK', StartPTUTrack(),
                transitions={'error':'failed',
                             'success':'START_CAMERA_TRACK' })
            smach.StateMachine.add('START_CAMERA_TRACK', StartCameraTrack(),
                transitions={'error':'failed',
                             'success':'TRAVEL_AROUND' })
            smach.StateMachine.add('TRAVEL_AROUND', TravelAroundObject(),
                transitions={'error':'STOP_CAMERA_TRACK',
                             'done':'STOP_CAMERA_TRACK' })
            smach.StateMachine.add('STOP_CAMERA_TRACK', StopCameraTrack(),
                transitions={'error':'failed',
                             'success':'STOP_PTU_TRACK' })
            smach.StateMachine.add('STOP_PTU_TRACK', StopPTUTrack(),
                transitions={'error':'failed',
                             'success':'STOP_TRANSFORM' })
            smach.StateMachine.add('STOP_TRANSFORM', StopSendingTransformation(),
                transitions={'error':'LEARN_MODEL',
                             'success':'LEARN_MODEL' })
            smach.StateMachine.add('LEARN_MODEL', LearnObjectModel(),
                transitions={'error':'succeeded',
                             'done':'succeeded' })
            
            self.set_initial_state(["METRIC_MAP_SWEEP"], userdata=self.userdata)
            self.register_termination_cb(self.finish)
            self._prior_movebase_config = {}
            self._prior_recoveries = {}
            self._reconfigure_client =  dynamic_reconfigure.client.Client(
                "/move_base/DWAPlannerROS")

            
    def execute(self, parent_ud = smach.UserData()):
        # Reduce the robot velocity
        self._prior_movebase_config = self._reconfigure_client.update_configuration({})
        self._reconfigure_client.update_configuration({'max_vel_x': 0.25,})

        # Disable recoveries on monitored navigation
        self._prior_recoveries =  rospy.get_param("/monitored_navigation/recover_states")
        recoveries =  copy.deepcopy(self._prior_recoveries)
        for r in recoveries.keys():
            if r == "sleep_and_retry":
                continue
            recoveries[r][0] = False
        rospy.set_param("/monitored_navigation/recover_states", recoveries)
        
        super(LearnObjectActionMachine, self).execute(parent_ud)
            
    
    def finish(self, userdata, terminal_states, outcome):
        print "Restoring monitored_nav & move_base parameters"
        # Restore movebase velocities
        self._reconfigure_client.update_configuration(self._prior_movebase_config)
        
        # Re-enable monitored nav recoveries
        rospy.set_param("/monitored_navigation/recover_states",
                        self._prior_recoveries)

        




if __name__ == '__main__':
    ''' Main Program '''
    rospy.init_node('test')
    SM =  LearnObjectActionMachine("WayPoint9")
    SM.execute()

    rospy.loginfo("All done.")
    rospy.spin()
