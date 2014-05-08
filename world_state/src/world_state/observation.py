import mongo
import rospy
import tf
from sensor_msgs.msg import Image, PointCloud2, CameraInfo, JointState
from geometry_msgs.msg import PoseWithCovarianceStamped
from ros_datacentre.message_store import MessageStoreProxy
from exceptions import StateException

DEFAULT_TOPICS = [("/amcl_pose", PoseWithCovarianceStamped),
                  ("/head_xtion/rgb/image_color", Image), 
                  ("/head_xtion/rgb/camera_info", CameraInfo), 
                  ("/head_xtion/depth/points", PointCloud2),
                  ("/head_xtion/depth/camera_info", CameraInfo),
                  ("/ptu/state", JointState)]

class MessageStoreObject(mongo.MongoTransformable):
    def __init__(self,  database="message_store", collection="message_store",
                 obj_id=None, typ=None):
        self.database = database
        self.collection = collection
        self.obj_id = obj_id
        self.typ = typ
        
    def retrieve(self):
        proxy = MessageStoreProxy(database=self.database,
                                  collection=self.collection)
        return proxy.query_id(self.obj_id, self.typ)[0]

class Observation(mongo.MongoTransformable):
    def __init__(self):
        self.stamp = rospy.Time.now().to_time()
        self._messages = {}
    
    @classmethod    
    def make_observation(cls, topics=DEFAULT_TOPICS):
        """
        topics: list of tuples (topic_name,topic_type)
        """
        observation = cls()
        message_proxy = MessageStoreProxy(collection="ws_observations")
        transforms = tf.TransformListener()
        for topic_name, topic_type in topics:
            rospy.loginfo("Aquiring message on %s [%s]"%(topic_name, topic_type._type))
            try:
                msg = rospy.wait_for_message(topic_name, topic_type, timeout=10.0)
            except rospy.ROSException, e:
                rospy.logwarn("Failed to get %s" % topic_name)
                continue
            msg_id = message_proxy.insert(msg)
            observation._messages[topic_name] = MessageStoreObject(
                database=message_proxy.database,
                collection=message_proxy.collection,
                obj_id=msg_id,
                typ=msg._type)
        return observation
    
    def get_message(self, topic):
        if not self._messages.has_key(topic):
            raise StateException("NO_OBSERVATION")
        return self._messages[topic].retrieve()