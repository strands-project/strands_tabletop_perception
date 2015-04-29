import rospy
import smach

from camera_srv_definitions.srv import start_tracker, stop_tracker, visualize_compound,get_tracking_results
from do_learning_srv_definitions.srv import learn_object, save_model
from recognition_srv_definitions.srv import get_configuration, recognize
from geometry_msgs.msg import Transform

from std_msgs.msg import String
from util import get_ros_service

class StartCameraTrack(smach.State):
    def __init__(self):
        smach.State.__init__( self, outcomes=['error', 'success'] )
        self._start_camera_tracker = get_ros_service("/camera_tracker/start_recording", start_tracker)       

    def execute(self, userdata):
        try:
            # start transformation
            self._start_camera_tracker()
            # need to wait for camera trackers initial frame drop to finish
            rospy.loginfo("Giving camera tracker 10s to init...")
            rospy.sleep(10)
            return "success"
        except:
            return "error"

class StopCameraTrack(smach.State):
    def __init__(self):
        smach.State.__init__( self, outcomes=['error', 'success'] )
        self._stop_camera_tracker = get_ros_service("/camera_tracker/stop_recording", stop_tracker)

    def execute(self, userdata):
        try:
            # stop transformation
            self._stop_camera_tracker()
            return "success"
        except:
            return "error"

class LearnObjectModel(smach.StateMachine):
    def __init__(self):
        smach.State.__init__( self, outcomes=['error', 'done'],
                              input_keys=['dynamic_object',] )
        self._get_tracking_results = get_ros_service("/camera_tracker/get_results", get_tracking_results)
        ## Object learning
        self._learn_object_model = get_ros_service("/dynamic_object_learning/learn_object", learn_object)
        self._save_object_model = get_ros_service("/dynamic_object_learning/save_model", save_model)

        ## Recognition services
        #self._get_rec_configuration = get_ros_service(
        #        "/recognition_service/get_configuration", get_configuration)
        #recognize_object = get_ros_service(
        #        "/recognition_service/mp_recognition", recognize)



    def execute(self, userdata):
        try:
            #rospy.sleep(20)
            tracks = self._get_tracking_results()
            rospy.loginfo("Got some tracking results!")
            rospy.loginfo("Number of keyframes:%d"%len(tracks.keyframes))
            rospy.loginfo("Transforms:")
            transforms=[Transform()]
            transforms[0].rotation.z=1
            transforms.extend(tracks.transforms)
            frames = [userdata.dynamic_object.object_cloud]
            frames.extend(tracks.keyframes)
            rospy.loginfo(tracks.transforms)
            print "About to call object learning, mask="
            #print userdata.dynamic_object.object_mask
            for f in frames:
                print "Frame width x height was: %d x %d. Changing it."%(f.width, f.height)
                f.width = 640
                f.height = 480
            self._learn_object_model(frames, transforms,
                                     userdata.dynamic_object.object_mask)
            rospy.loginfo("Saving learnt model..")
            self._save_object_model(String("experiment27"),
                                    String("/home/strands/test_models"),
                                    String("/home/strands/test_models/reconstruct"))
            return 'done'
        except Exception, e:
            print e
            return "error"
