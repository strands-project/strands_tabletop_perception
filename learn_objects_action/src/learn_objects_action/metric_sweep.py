import rospy
import smach
import actionlib
import numpy
from geometry_msgs.msg import Pose

from scitos_ptu.msg import PanTiltAction, PanTiltActionGoal,PanTiltGoal
from object_manager.srv import DynamicObjectsService, GetDynamicObjectService
from object_view_generator.srv import GetTrajectoryPoints, GetTrajectoryPointsResponse
from std_msgs.msg import String
from util import get_ros_service
from semantic_map_to_2d.srv import ChangeWaypoint
import yaml
from world_state.observation import MessageStoreObject, Observation, TransformationStore
from world_state.identification import ObjectIdentification
from world_state.state import World, Object
import os

class MetricSweep(smach.State):
    def __init__(self):
        smach.State.__init__( self, outcomes=['done', 'failed', 'preempted'], 
                              input_keys=['action_goal'] )

        self._metric_sweep_action = actionlib.SimpleActionClient(
            '/ptu_pan_tilt_metric_map',
            PanTiltAction)
        rospy.loginfo("Initialising LeanObjects SM: "
                      "Waiting for metric map action server...")
        if not self._metric_sweep_action.wait_for_server(rospy.Duration(5)):
            rospy.logerr("Metric map action can't be found!")
            raise Exception("LearnObject SM can't initialise; missing service.")
        rospy.loginfo("-> got metric map ptu action.")

        self._set_waypoint_map = get_ros_service("/set_waypoint",
                                      ChangeWaypoint)


        self._goal = PanTiltGoal()
        # fill in the goal...
        self._goal.pan_start = -160
        self._goal.pan_step = 20
        self._goal.pan_end = 160
        self._goal.tilt_start = -30
        self._goal.tilt_step = 30
        self._goal.tilt_end = 30

    def execute(self, userdata):
        try:
            self._metric_sweep_action.send_goal(self._goal)
            while not self._metric_sweep_action.wait_for_result(rospy.Duration(0.5)):
                if self.preempt_requested():
                    self.service_preempt()
                    break
            else:
                # wait for the message that it has been processed...
                try:
                    status = rospy.wait_for_message("/local_metric_map/status", String,
                                                    timeout=120)
                except rospy.ROSException,  e:
                    rospy.logwarn("Never got a message to say that a metric "
                                  "map was processed. Learn objects action failed.")
                    return 'failed'
                if status.data !=  "finished_processing_observation":
                    return 'failed'
                
                # Update the local waypoint 2d map
                resp =  self._set_waypoint_map(userdata.action_goal.waypoint)
                if not resp.is_ok:
                    return 'failed'
                return "done"
            rospy.loginfo("Premempting metric sweep.")
            self._metric_sweep_action.cancel_goal()
            self._metric_sweep_action.wait_for_result()
            return "preempted"

        except Exception, e:
            rospy.logwarn("Failed to perform a metric sweep")
            rospy.logwarn("-> Exceptions was: %s" % str(e))
            return "failed"


class SelectCluster(smach.State):
    def __init__(self):
        smach.State.__init__( self, outcomes=['selected', 'none'],
                              input_keys=['action_goal','object'],
                              #input_keys=['waypoint'],
                              output_keys=['dynamic_object', 'dynamic_object_centroid',
                                           'dynamic_object_points','dynamic_object_id', 'object'])
        self._get_clusters = get_ros_service("/object_manager_node/ObjectManager/DynamicObjectsService",
                                             DynamicObjectsService)
        self._get_specific_cluster = get_ros_service("/object_manager_node/ObjectManager/GetDynamicObjectService",
                                                     GetDynamicObjectService)
        self._get_plan_points = get_ros_service('/test_nav_goal', GetTrajectoryPoints)

    def execute(self, userdata):
        # Load the waypoint to soma from file, ugly ugly ugly TODO: properly
        try:
            with open(os.path.expanduser("~/.waypointsomas"), "r") as f:
                somas = yaml.load(f.read())	
            soma_region = somas[userdata.action_goal.waypoint]
            #clusters = self._get_clusters(userdata['waypoint'])
            clusters = self._get_clusters(userdata.action_goal.waypoint)
            if len(clusters.object_id) == 0:
                return 'none'
            # which cluster should we choose!!??
            ID=2

            scores = [0] * len(clusters.object_id)
            for i, pnt in enumerate(clusters.centroids):
                p =  Pose()
                p.position.x = pnt.x
                p.position.y = pnt.y
                poses = self._get_plan_points(min_dist=1.0, 
                                              max_dist=1.5,
                                              number_views=25,
                                              inflation_radius=0.3, 
                                              target_pose=p,
                                                                  SOMA_region=soma_region)
                scores[i] = len(poses.goals.poses)
            ID=numpy.argmax(scores) # the one to look at is the one that has the most observation available
            rospy.logwarn( "Getting cluster: %s"%clusters.object_id[ID])
            #one_cluster = self._get_specific_cluster(userdata['waypoint'], clusters.object_id[ID])
            one_cluster = self._get_specific_cluster(userdata.action_goal.waypoint,
                                                     clusters.object_id[ID])
            userdata['dynamic_object']=one_cluster
            userdata.dynamic_object_centroid = clusters.centroids[ID]
            userdata.dynamic_object_points = clusters.objects[ID]
            userdata.dynamic_object_id = clusters.object_id[ID]
            world = World()
            userdata['object'] = world.create_object()
            userdata['object'].add_identification('ObjectLearnerID', ObjectIdentification({'NEW':1}))
            print clusters.object_id[ID]
            print "="*20
            print clusters.centroids[ID]
            print "="*20
            return "selected"
        except Exception, e:
            rospy.logwarn("Failed to select a cluster!!!!")
            rospy.logwarn("-> Exceptions was: %s" % str(e))
            return "none"
        
        
