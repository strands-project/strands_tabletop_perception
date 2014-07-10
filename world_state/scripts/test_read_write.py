#!/usr/bin/python
import rospy
from world_state.observation import TransformationStore, Observation
from world_state.mongo import MongoConnection
from sensor_msgs.msg import PointCloud2,  JointState
from geometry_msgs.msg import PoseWithCovarianceStamped
from recognition_srv_definitions.srv import *

if __name__ == '__main__':
    ''' Main Program '''
    rospy.init_node("observation_tester")

    mongodb =  MongoConnection(database_name="snapshots")

    # Make an 'Observation' object that grabs ROS topics and hold an index
    # to their MessageStore saved messages.
    TOPICS = [("/amcl_pose", PoseWithCovarianceStamped),
              ("/head_xtion/depth/points", PointCloud2),
              ("/ptu/state", JointState) ]
    observation = Observation.make_observation(TOPICS)

    # Save the observation to the datacentre
    observation_id = mongodb.database.observations.insert({'observation': observation,
                                                           'observation_name': 'test',})

    #===

    # Load an observation from the datacentre
    observation = mongodb.database.observations.find_one({'observation_name': 'test'})['observation']

    # Get the messages that were part of the observation. This directly gives
    # a ROS message - here a sensor_msgs/PointCloud2 and geometry_msgs/PoseWithCovarianceStamped
    pointcloud = observation.get_message("/head_xtion/depth/points")
    pose = observation.get_message("/amcl_pose")

    # Reading the transformation tree is done by reconstructing a ROS transformer
    # that then works in the same way as a TF Listener object.
    tf =  TransformationStore.msg_to_transformer(observation.get_message("/tf"))
    print tf.lookupTransform("/map", "/base_footprint", rospy.Time(0))

    rospy.wait_for_service("/multiview_object_recognizer_node/multiview_recognotion_servcice");
    try:
	mv_client = rospy.ServiceProxy("/multiview_object_recognizer_node/multiview_recognotion_servcice", multiview_recognize)
    	resp1 = mv_client(pointcloud, "scene1")
    except rospy.ServiceException, e:
	print "Service call failed: %s"%e
