import mongo
import rospy
import tf
from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import PoseWithCovarianceStamped
from ros_datacentre.message_store import MessageStoreProxy

DEFAULT_TOPICS = [("/amcl_pose", PoseWithCovarianceStamped),
                  ("/chest_xtion/rgb/image_color", Image), 
                  ("/chest_xtion/depth/points", PointCloud2)]

class MessageStoreObject(mongo.MongoTransformable):
    def __init__(self,  database="message_store", collection="message_store",
                 obj_id=None, typ=None):
        self.database = database
        self.collection = collection
        self.obj_id = obj_id
        self.typ = typ

class Observation(mongo.MongoTransformable):
    # Snap shot of robot sensors & actuators
    # What:
    #  - TF
    #  - RGB Image
    #  - Point cloud
    #  - indices
    #  - stmp
    def __init__(self):
        self.stamp = rospy.Time.now().to_time()
        self._messages = {}
    
    @classmethod    
    def make_observation(cls, topics):
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