#!/usr/bin/python
import rospy
import actionlib

from world_state.observation import TransformationStore, Observation
from world_state.mongo import MongoConnection
from sensor_msgs.msg import PointCloud2,  JointState, Image, CameraInfo
from geometry_msgs.msg import PoseWithCovarianceStamped
from recognition_srv_definitions.srv import *
from table_segmentation.srv import *
from scene_recorder.msg import *
from std_msgs.msg import String, Time
import world_state.geometry as geometry

if __name__ == '__main__':
		rospy.init_node('replay_snapshot_node')
		scene_name = rospy.get_param('~scene_name', 'table1')
		session_id = rospy.get_param('~session_id', 1)
		print "Replaying for scene: " + scene_name + " and session id: " + `session_id`
                
		mongodb =  MongoConnection(database_name="snapshots")

                # Make an 'Observation' object that grabs ROS topics and hold an index
                # to their MessageStore saved messages.
                TOPICS = [("/amcl_pose", PoseWithCovarianceStamped),
                          ("/head_xtion/rgb/image_raw", Image),
                          ("/head_xtion/rgb/camera_info", CameraInfo),
                          ("/head_xtion/depth_registered/points", PointCloud2),
                          ("/ptu/state", JointState) ]

                # Load an observation from the datacent
                view_id = 0;

                while True:
                                observation_id = scene_name + "_" + `session_id` + "_" + `view_id`
                                print "retrieving observation : " + observation_id

                                observation = mongodb.database.observations.find_one({'observation_name': observation_id})['observation']
                                if observation == False:
                                        break

                                # Get the messages that were part of the observation. This directly gives
                                # a ROS message - here a sensor_msgs/PointCloud2 and geometry_msgs/PoseWithCovarianceStamped
                                pointcloud = observation.get_message("/head_xtion/depth_registered/points")
                                pose = observation.get_message("/amcl_pose")
				print pointcloud.width
				print pointcloud.height
                                # Reading the transformation tree is done by reconstructing a ROS transformer
                                # that then works in the same way as a TF Listener object.
                                tf =  TransformationStore.msg_to_transformer(observation.get_message("/tf"))
                                timestamp = observation.get_message("/head_xtion/depth_registered/points").header.stamp
                                print "Timestamp: " + `timestamp`
                                transform = tf.lookupTransform("/map", pointcloud.header.frame_id, timestamp)
                                print "Transform: " + str(transform)
                                depth_to_world = geometry.Pose(geometry.Point(*(transform[0])), geometry.Quaternion(*(transform[1])))
                                trans_homo =  depth_to_world.as_homog_matrix()
                                print trans_homo

                                transf_1d = [0]*16
                                for row in range(0,4):
                                        for col in range(0,4):
                                                transf_1d[4*row + col] = trans_homo[row][col]

				#rospy.wait_for_service("/segment_table")
                                #try:
                                #        table_segment_client = rospy.ServiceProxy("/segment_table", SegmentTable)
                                #        resp = table_segment_client(pointcloud, scene_name, transf_1d)
                               # 	pointcloud = resp.cloud
			#	except rospy.ServiceException, e:
        #                #               print "Service call failed: %s"%e
                          #              print "Table segmentation service call failed"


                                rospy.wait_for_service("/multiview_object_recognizer_node/multiview_recognotion_servcice");
                                try:
                                        mv_client = rospy.ServiceProxy("/multiview_object_recognizer_node/multiview_recognotion_servcice", multiview_recognize)
                                        msg_to_send = multiview_recognize()
                                        msg_to_send.cloud = pointcloud
					view_id_string = "%05d" % view_id
					print view_id_string
                                        msg_to_send.scene_name = String(scene_name)
                                        msg_to_send.view_name = String(view_id_string)
                                        msg_to_send.transform = transf_1d
                                        #msg_to_send.timestamp = Int32(timestampe)
                                        resp1 = mv_client(pointcloud, String(scene_name), String(view_id_string), transf_1d, Time(timestamp))
                                	
					print "Using multi-view recognition, following objects were detected: "

					for id in range (0, len(resp1.ids)):
						print resp1.ids[id].data

				except rospy.ServiceException, e:
        #                      		print "Service call failed: %s"%e
                                        print "Service call failed"

                                view_id = view_id + 1
