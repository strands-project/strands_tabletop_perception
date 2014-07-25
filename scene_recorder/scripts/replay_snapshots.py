#!/usr/bin/python
import rospy
import actionlib

from world_state.observation import TransformationStore, Observation
from world_state.mongo import MongoConnection
from sensor_msgs.msg import PointCloud2,  JointState, Image, CameraInfo
from geometry_msgs.msg import PoseWithCovarianceStamped
from recognition_srv_definitions.srv import *
from scene_recorder.msg import *
from std_msgs.msg import String

if __name__ == '__main__':
		rospy.init_node("TakeSnapshot")
                mongodb =  MongoConnection(database_name="snapshots", server="romeo")

                # Make an 'Observation' object that grabs ROS topics and hold an index
                # to their MessageStore saved messages.
                TOPICS = [("/amcl_pose", PoseWithCovarianceStamped),
                          ("/head_xtion/rgb/image_color", Image),
                          ("/head_xtion/rgb/camera_info", CameraInfo),
                          ("/head_xtion/depth_registered/points", PointCloud2),
                          ("/ptu/state", JointState) ]

                # Load an observation from the datacentre
                observation = mongodb.database.observations.find_one({'observation_name': 'table13'})['observation']

                # Get the messages that were part of the observation. This directly gives
                # a ROS message - here a sensor_msgs/PointCloud2 and geometry_msgs/PoseWithCovarianceStamped
                pointcloud = observation.get_message("/head_xtion/depth_registered/points")
                pose = observation.get_message("/amcl_pose")

                # Reading the transformation tree is done by reconstructing a ROS transformer
                # that then works in the same way as a TF Listener object.
                tf =  TransformationStore.msg_to_transformer(observation.get_message("/tf"))
                print tf.lookupTransform("/map", "/base_footprint", rospy.Time(0))


                rospy.wait_for_service("/multiview_object_recognizer_node/multiview_recognotion_servcice");
                try:
                        mv_client = rospy.ServiceProxy("/multiview_object_recognizer_node/multiview_recognotion_servcice", multiview_recognize)
                        #view_name = `self._recorded_snapshots`
                        view_name = 'test'
			resp1 = mv_client(pointcloud, String('scene1'), String(view_name))
                except rospy.ServiceException, e:
                        print "Service call failed: %s"%e

		rospy.spin()
