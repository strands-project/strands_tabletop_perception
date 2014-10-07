#!/usr/bin/python
import rospy
import actionlib

from world_state.observation import TransformationStore, Observation
from world_state.mongo import MongoConnection
from sensor_msgs.msg import PointCloud2,  JointState, Image, CameraInfo
from geometry_msgs.msg import PoseWithCovarianceStamped
from recognition_srv_definitions.srv import *
from scene_recorder.msg import *
from std_msgs.msg import String, Time, Int32

class TakeSnapshotActionServer(object):
	_feedback = TakeSnapshotFeedback()
	_result = TakeSnapshotResult()
	

	def send_feedback(self, txt):
		self._feedback.status = txt
		self._as.publish_feedback(self._feedback)
		rospy.loginfo(txt)

	def __init__(self, name):
		self._action_name = name
		self._as = actionlib.SimpleActionServer(self._action_name, TakeSnapshotAction, execute_cb=self.execute_cb, auto_start = False)
		self._as.start()
		self._recorded_snapshots = 0
		rospy.loginfo("Action server up: %s"%self._action_name)

	def execute_cb(self, goal):
		observation_name = goal.scene_name + "_" + `goal.session_id` + "_" + `self._recorded_snapshots`
		self.send_feedback("Connecting to MongoDB")
		#mongodb =  MongoConnection(database_name="snapshots")

		self.send_feedback("Taking snapshot with name %s and store it into mongodb"%observation_name)
    		# Make an 'Observation' object that grabs ROS topics and hold an index
    		# to their MessageStore saved messages.
		TOPICS = [("/head_xtion/rgb/image_raw", Image), 
			  ("/head_xtion/rgb/camera_info", CameraInfo), 
        		  ("/head_xtion/depth_registered/points", PointCloud2) ]
		observation = Observation.make_observation(TOPICS)

		pointcloud = observation.get_message("/head_xtion/depth_registered/points")

		print pointcloud.width
		print pointcloud.height
		timestamp = observation.get_message("/head_xtion/depth_registered/points").header.stamp
		print "Timestamp: " + `timestamp`

        	rospy.wait_for_service("/multiview_object_recognizer_node/multiview_recognotion_service");

		try:
			print 'blablabla'
			mv_client = rospy.ServiceProxy("/multiview_object_recognizer_node/multiview_recognotion_service", multiview_recognize)
			msg_to_send = multiview_recognize()
			msg_to_send.cloud = pointcloud
			view_id_string = "%05d" % self._recorded_snapshots
			print view_id_string
			transf_1d = []
			msg_to_send.scene_name = String("test")
			msg_to_send.view_name = String(view_id_string)
			#msg_to_send.transform = transf_1d
			msg_to_send.timestamp = Int32(timestamp)
			resp1 = mv_client(pointcloud, String("test"), String(view_id_string), transf_1d, Time(timestamp))
		except rospy.ServiceException, e:
			print "Error calling multi-view recgonizer" 

		print "Using multi-view recognition, following objects were detected: "

		for id in range (0, len(resp1.ids)):
			print resp1.ids[id].data

		#rospy.loginfo("saving %s"%observation_name)
   		# Save the observation to the datacentre
		#observation_id = mongodb.database.observations.insert({'observation': observation, 'observation_name': observation_name})
		self._result.view_id = self._recorded_snapshots
		self._recorded_snapshots += 1
		self._as.set_succeeded(self._result)

if __name__ == '__main__':
	rospy.init_node("TakeSnapshotAndRecognize")
	print "Hallo"
	TakeSnapshotActionServer(rospy.get_name())
	rospy.spin()
