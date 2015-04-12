#!/usr/bin/env python
# import roslib; roslib.load_manifest('nav_goals_generator')
import rospy
import tf

from geometry_msgs.msg import PoseStamped,TransformStamped
from geometry_msgs.msg import Pose

import actionlib
from camera_srv_definitions.srv import start_tracker, stop_tracker, visualize_compound,get_tracking_results
from do_learning_srv_definitions.srv import learn_object, save_model
from recognition_srv_definitions.srv import get_configuration, recognize
from scitos_ptu.msg import PanTiltAction, PanTiltActionGoal,PanTiltGoal
from  strands_navigation_msgs.msg import MonitoredNavigationAction,MonitoredNavigationGoal
from ptu_follow_frame.srv import SetTransformation, StopTransformation, StartFollowing
from std_srvs.srv import Empty
from nav_goals_generator.srv import NavGoals, NavGoalsResponse
from flir_pantilt_d46.srv import SetControlMode
from object_manager.srv import DynamicObjectsService
import sys

from sensor_msgs.msg import Image

rospy.init_node("learn_object")
debug_image_pub = rospy.Publisher("/object_learning/debug_image", Image, queue_size=1)

def get_ros_service(srv_name, srv_type):
        try:
                rospy.loginfo("Getting '%s' service.."%srv_name)
                rospy.wait_for_service(srv_name, 5)
                rospy.loginfo("ok.")
        except:
                print "Service '%s' not available. Aborting"%srv_name
                sys.exit(1)
        return rospy.ServiceProxy(srv_name, srv_type)


## Camera tracking services
#start_camera_tracker = get_ros_service("/camera_tracker/start_recording", start_tracker)
#stop_camera_tracker = get_ros_service("/camera_tracker/stop_recording", stop_tracker)
#visualize_tracker = get_ros_service("/camera_tracker/vis_compound", visualize_compound)
#tracking_results = get_ros_service("/camera_tracker/get_results", get_tracking_results)
## Object learning
#learn_object_model = get_ros_service("/dynamic_object_learning/learn_object", learn_object)
#save_object_model = get_ros_service("/dynamic_object_learning/save_model", save_model)

## Recognition services
#get_rec_configuration = get_ros_service(
#        "/recognition_service/get_configuration", get_configuration)
#recognize_object = get_ros_service(
#        "/recognition_service/mp_recognition", recognize)

# Metric map / dynamic cluster services
def do_sweep():
        client = actionlib.SimpleActionClient('/ptu_pan_tilt_metric_map',
                                              PanTiltAction)
        rospy.loginfo("Waiting for metric map action server...")
        client.wait_for_server()
        rospy.loginfo("Got it. Calling it...")
        goal = PanTiltGoal()
        # fill in the goal...
        goal.pan_start = -160
        goal.pan_step = 20
        goal.pan_end = 160
        goal.tilt_start = -30
        goal.tilt_step = 30
        goal.tilt_end = 30
        client.send_goal(goal)
        client.wait_for_result()
        rospy.loginfo("ok.")

get_clusters = get_ros_service("/object_manager_node/ObjectManager/DynamicObjectsService", DynamicObjectsService)


# Transformation services
set_transform = get_ros_service("/static_transforms_manager/set_tf", SetTransformation)
stop_transform = get_ros_service("/static_transforms_manager/stop_tf", StopTransformation)

# PTU tracking services
set_control_mode = get_ros_service("/ptu/set_control_mode", SetControlMode)
set_tracking_frame = get_ros_service("/ptu_follow_frame/set_following",StartFollowing)
stop_tracking_frame = get_ros_service("/ptu_follow_frame/stop_following",Empty)

# Robot moving services
def observe_point(x, y):
	rospy.loginfo("Going to observe a point....")
        client = actionlib.SimpleActionClient('/monitored_navigation', MonitoredNavigationAction)
        rospy.loginfo("Waiting for monitored_nav...")
        client.wait_for_server()
        rospy.loginfo("Got it.")
        
	p =  Pose()
	p.position.x = x
	p.position.y = y
	srv =  rospy.ServiceProxy('/test_nav_goal', NavGoals)
	poses = srv(1.5, 2, 20, 0.5, p)
        for i,p in enumerate(poses.goals.poses):
                print i+1,"/",len(poses.goals.poses)
                targ = PoseStamped()
                targ.header.frame_id="/map"
                targ.pose=p
                goal = MonitoredNavigationGoal("move_base", targ)
                client.send_goal(goal)
                print client.wait_for_result()

                
                print "Grabbing an image for debug"
                rospy.sleep(5)
                img = rospy.wait_for_message("/head_xtion/rgb/image_color", Image)
                debug_image_pub.publish(img)
                

############### OK #################
##### Let's do this thing. #########

CURRENT_WAYPOINT="WayPoint9"
#do_sweep()

clusters = get_clusters(CURRENT_WAYPOINT)

# choose one
ID=0
one = clusters.object_id[ID]
#chosen = get_clusters(one)
one_point = clusters.centroids[ID]

#start_tracker()
trans = TransformStamped()

trans.transform.translation.x = one_point.x
trans.transform.translation.y = one_point.y
trans.transform.translation.z = one_point.z
trans.transform.rotation.w = 1
trans.header.frame_id = "map"
trans.child_frame_id = "cluster"

set_transform(trans)
set_control_mode(1)
set_tracking_frame("cluster")

observe_point( one_point.x, one_point.y )

stop_tracking_frame()
set_control_mode(0)

#results = visualize_tracker()

#learn_object(results)

#conf = get_rec_configuration()
#save_model(conf)

rospy.spin()

