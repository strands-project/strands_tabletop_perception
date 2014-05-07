#!/usr/bin/python
import rospy
from world_state.state import World, Object
from world_state.identification import ObjectIdentification
from world_state.observation import MessageStoreObject, Observation, DEFAULT_TOPICS
from world_state.geometry import Pose
from sensor_msgs.msg import Image, PointCloud2

import world_state.objectmaster as objectmaster

from ros_datacentre.message_store import MessageStoreProxy

from strands_perception_msgs.msg import Table

if __name__ == '__main__':
    ''' Main Program '''
    rospy.init_node("observation_tester")
    pub = rospy.Publisher("git_image", Image, latch=1)
    
    w = World()
    om =  objectmaster.ObjectMaster()

    rospy.loginfo("Creating object.")
    ob = w.create_object()
    observation = Observation.make_observation(DEFAULT_TOPICS)
    ob.add_observation(observation)
    
    image =  observation.get_message("/chest_xtion/rgb/image_color")
    
    pub.publish(image)
    