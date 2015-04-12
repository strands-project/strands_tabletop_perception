import rospy
import smach
import math

from std_srvs.srv import Empty
from geometry_msgs.msg import TransformStamped
from util import get_ros_service
from sensor_msgs.msg import JointState
from static_transform_manager.srv import SetTransformation, StopTransformation
from ptu_follow_frame.srv import StartFollowing

class TurnPTUToObject(smach.State):
    def __init__(self):
        smach.State.__init__( self, outcomes=['error', 'success'],
                              input_keys=['dynamic_object'] )

        self._ptu_angle_pub = rospy.Publisher("/ptu/cmd", JointState)
        self._jnts = JointState()
        self._jnts.name=["pan","tilt"]
        self._jnts.velocity=[1,1]
        self._jnts.position=[0,0]

    def execute(self, userdata):
        try:
            # start transformation
            self._jnts.position=[math.radians(userdata.dynamic_object.pan_angle),
                                 math.radians(userdata.dynamic_object.tilt_angle)]
            self._ptu_angle_pub.publish(self._jnts)
            rospy.sleep(3)
            return "success"
        except Exception, e:
            print e
            return "error"

class StartTransformation(smach.State):
    def __init__(self):
        smach.State.__init__( self, outcomes=['error', 'success'],
                              input_keys=['dynamic_object_centroid'])

        self._set_transform = get_ros_service("/static_transforms_manager/set_tf",
                                              SetTransformation)


    def execute(self, userdata):
        try:
            # stop transformation
            trans = TransformStamped()

            trans.transform.translation.x = userdata.dynamic_object_centroid.x
            trans.transform.translation.y = userdata.dynamic_object_centroid.y
            trans.transform.translation.z = userdata.dynamic_object_centroid.z
            trans.transform.rotation.w = 1
            trans.header.frame_id = "map"
            trans.child_frame_id = "cluster"
            self._set_transform(trans)
            return "success"
        except Exception, e:
            print e
            return "error"

class StopSendingTransformation(smach.State):
    def __init__(self):
        smach.State.__init__( self, outcomes=['error', 'success'],
                              input_keys=['dynamic_object_centroid'])
        self._stop_transform = get_ros_service("/static_transforms_manager/stop_tf",
                                               StopTransformation)


    def execute(self, userdata):
        try:
            # stop transformation
            self._stop_transform("cluster")
            return "success"
        except Exception, e:
            print e
            return "error"

class StartPTUTrack(smach.State):
    def __init__(self):
        smach.State.__init__( self, outcomes=['error', 'success'] )
        self._set_tracking_frame = get_ros_service("/ptu_follow_frame/set_following",
                                                   StartFollowing)

    def execute(self, userdata):
        try:
            # start transformation
            self._set_tracking_frame("cluster")
            return "success"
        except:
            return "error"

class StopPTUTrack(smach.State):
    def __init__(self):
        smach.State.__init__( self, outcomes=['error', 'success'] )
        self._stop_tracking_frame = get_ros_service("/ptu_follow_frame/stop_following",
                                                    Empty)

    def execute(self, userdata):
        try:
            # stop transformation
            self._stop_tracking_frame()
            return "success"
        except:
            return "error"
