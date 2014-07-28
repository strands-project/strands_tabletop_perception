#!/usr/bin/python
import rospy
import actionlib

from std_msgs.msg import *
from sensor_msgs.msg import *


from world_state.observation import TransformationStore, Observation
from world_state.mongo import MongoConnection
from sensor_msgs.msg import PointCloud2,  JointState, Image, CameraInfo
from geometry_msgs.msg import PoseWithCovarianceStamped
from recognition_srv_definitions.srv import *
from scene_recorder.msg import *
from std_msgs.msg import String
import world_state.geometry as geometry

if __name__ == '__main__':
		rospy.init_node("TakeSnapshot")
                mongodb =  MongoConnection(database_name="snapshots")

                # Make an 'Observation' object that grabs ROS topics and hold an index
                # to their MessageStore saved messages.
                TOPICS = [("/amcl_pose", PoseWithCovarianceStamped),
                          ("/head_xtion/rgb/image_color", Image),
                          ("/head_xtion/rgb/camera_info", CameraInfo),
                          ("/head_xtion/depth_registered/points", PointCloud2),
                          ("/ptu/state", JointState) ]

                # Load an observation from the datacentre

		scene_name = 'table2_'
		view_id = 8;
		
		while True:
		                observation_id = scene_name + `view_id`
		                print observation_id

				observation = mongodb.database.observations.find_one({'observation_name': observation_id})['observation']
				if observation == False:
					break

		            	# Get the messages that were part of the observation. This directly gives
		           	# a ROS message - here a sensor_msgs/PointCloud2 and geometry_msgs/PoseWithCovarianceStamped
		            	pointcloud = observation.get_message("/head_xtion/depth_registered/points")
		            	pose = observation.get_message("/amcl_pose")

		            	# Reading the transformation tree is done by reconstructing a ROS transformer
		            	# that then works in the same way as a TF Listener object.
		            	tf =  TransformationStore.msg_to_transformer(observation.get_message("/tf"))
                                transform = tf.lookupTransform("/map", pointcloud.header.frame_id, observation.get_message("/head_xtion/rgb/camera_info").header.stamp)
                                print "Transform: " + str(transform)
            			depth_to_world = geometry.Pose(geometry.Point(*(transform[0])), geometry.Quaternion(*(transform[1])))
				trans_homo =  depth_to_world.as_homog_matrix()
				print trans_homo
				
				transf_1d = [0]*16
				for row in range(0,4):
					for col in range(0,4):
						transf_1d[4*row + col] = trans_homo[row][col]
	
		            	rospy.wait_for_service("/multiview_object_recognizer_node/multiview_recognotion_servcice");
		            	try:
		                	mv_client = rospy.ServiceProxy("/multiview_object_recognizer_node/multiview_recognotion_servcice", multiview_recognize)	
					resp1 = mv_client(pointcloud, String(scene_name), String(`view_id`), transf_1d)
		           	except rospy.ServiceException, e:
	#                      		print "Service call failed: %s"%e
					print "Service call failed"
				view_id = view_id + 1
		#rospy.spin()
