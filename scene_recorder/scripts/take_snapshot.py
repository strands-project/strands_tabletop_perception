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

class TakeSnapshotActionServer(object):
	_feedback = TakeSnapshotFeedback()
	_result = TakeSnapshotResult()
	
	def __init__(self, name):
		self._action_name = name
		self._as = actionlib.SimpleActionServer(self._action_name, TakeSnapshotAction, execute_cb=self.execute_cb, auto_start = False)
		self._as.start()
		self._recorded_snapshots = 0
		rospy.loginfo("Action server up: %s"%self._action_name)

	def execute_cb(self, goal):
		observation_name = goal.scene_name + `self._recorded_snapshots`
	    	self.send_feedback("Connecting to MongoDB")
		mongodb =  MongoConnection(database_name="snapshots", server="romeo")

                self.send_feedback("Taking snapshot with name %s and store it into mongodb"%observation_name)
    		# Make an 'Observation' object that grabs ROS topics and hold an index
    		# to their MessageStore saved messages.
    		TOPICS = [("/amcl_pose", PoseWithCovarianceStamped),
			  ("/head_xtion/rgb/image_color", Image), 
			  ("/head_xtion/rgb/camera_info", CameraInfo), 
        		  ("/head_xtion/depth_registered/points", PointCloud2),
            	          ("/ptu/state", JointState) ]
		observation = Observation.make_observation(TOPICS)

		rospy.loginfo("saving %s"%observation_name)
   		# Save the observation to the datacentre
    		observation_id = mongodb.database.observations.insert({'observation': observation,
                                                           'observation_name': observation_name,})
	    	self._recorded_snapshots += 1
		

		self.send_feedback('Loading observation from the database')
		# Load an observation from the datacentre
    		observation = mongodb.database.observations.find_one({'observation_name': observation_name})['observation']

    		# Get the messages that were part of the observation. This directly gives
    		# a ROS message - here a sensor_msgs/PointCloud2 and geometry_msgs/PoseWithCovarianceStamped
    		pointcloud = observation.get_message("/head_xtion/depth_registered/points")
    		pose = observation.get_message("/amcl_pose")

    		# Reading the transformation tree is done by reconstructing a ROS transformer
    		# that then works in the same way as a TF Listener object.
#		self.send_feedback('Looking up transform')
  		tf =  TransformationStore.msg_to_transformer(observation.get_message("/tf"))
   		print tf.lookupTransform("/map", "/base_footprint", rospy.Time(0))


#        	rospy.wait_for_service("/multiview_object_recognizer_node/multiview_recognotion_servcice");
 #      		try:
  #              	mv_client = rospy.ServiceProxy("/multiview_object_recognizer_node/multiview_recognotion_servcice", multiview_recognize)
#			view_name = `self._recorded_snapshots`
 #               	resp1 = mv_client(pointcloud, String(goal.scene_name), String(view_name))
  #    			#self._result.found = 1
   #   			#self._as.set_succeeded(self._result)
#
#		except rospy.ServiceException, e:
 #               	print "Service call failed: %s"%e
#			self.send_feedback('Couldn\'t call multiview_recognition_service')
 #     			#self._result.found = 0
      			#self._as.set_succeeded(self._result)

	def send_feedback(self, txt):
		self._feedback.status = txt
		self._as.publish_feedback(self._feedback)
		rospy.loginfo(txt)

if __name__ == '__main__':
	rospy.init_node("TakeSnapshot")
	print "Hallo"
	TakeSnapshotActionServer(rospy.get_name())
	rospy.spin()
