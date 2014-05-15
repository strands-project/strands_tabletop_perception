import rospy
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
import math

from state import World, Object

# Import opencv
import cv
import cv2
from sensor_msgs.msg import Image, CameraInfo
from sensor_msgs.msg import PointCloud, PointCloud2
import sensor_msgs.point_cloud2 as pc2
from cv_bridge import CvBridge, CvBridgeError
from image_geometry import PinholeCameraModel
import numpy as np
import geometry

from observation import MessageStoreObject, Observation, TransformationStore

from ros_datacentre.message_store import MessageStoreProxy
from robblog.msg import RobblogEntry
import robblog.utils
import datetime

def generate_report(t, parent_object=None):
    """
    parent_object: string, parent id, only children of this are included
    t: time, float
    """
    # what objects existed at time t 
    w = World()
    if parent_object is None:
        objects = w.get_root_objects()
    else:
        objects = [w.get_object(parent_object)]
        
    for o in objects:
        print "-> ", o.name, " ==> ", o.identification.class_type
        for o2 in [w.get_object(ob) for ob in o.get_children_names()]:
            print "-> -> ", o2.name, " ==> ", o2.identification.class_type

def create_table_observation_image(table_name, observation_timestamp):
    """
    returns numpy/opencv image
    """
    # what objects existed on the table at this timestamp..
    w = World()
    table =  w.get_object(table_name)
    # children = w.get_children(table_name, {'$and': [
                                   #{'_life_start': {'$lt': timestamp} },
                                    #{'$or': [{'_life_end': {'$gt': timestamp}},
                                            #{'_life_end': None } ]}
                                    #] })
    # Which table observation is closest to timestamp
    if len(table._observations) < 1:
        raise Exception("Table has no observations")
    closest =  min([(ob, math.fabs(ob.stamp - observation_timestamp)) for ob in table._observations],
                   key=lambda x: x[1])
    rospy.loginfo("Clost observation of table '%s' to time %d was %d"%(table_name,
                                                                        observation_timestamp,
                                                                        closest[1]))
    observation = closest[0]
    tf =  TransformationStore.msg_to_transformer(observation.get_message("/tf"))
    camera_info =  observation.get_message("/head_xtion/rgb/camera_info")
    bridge = CvBridge()
    rgb_image = bridge.imgmsg_to_cv2(observation.get_message("/head_xtion/rgb/image_color"))
          
    children = w.get_children(table_name, {'_observations': {'$elemMatch': {'stamp': observation.stamp}}})
    colours = [(255, 0, 0), (0, 255, 0), (0, 0, 255), (255, 0, 255),
               (0, 255, 255), (255, 255, 0)   ]
    for j, c in enumerate(children):
        print c.name, ' ==> ', c.identification.class_type
        pointcloud = c._point_cloud.retrieve()
        print c._point_cloud.obj_id
        world_to_rgb = tf.lookupTransform(camera_info.header.frame_id, "/map", 
                                                pointcloud.header.stamp)
        world_to_rgb = geometry.Pose(geometry.Point(*(world_to_rgb[0])),
                                   geometry.Quaternion(*(world_to_rgb[1])))
        transform = np.dot(world_to_rgb.as_homog_matrix(),
                           table.pose.as_homog_matrix() )
        transform = np.dot(transform, c.pose.as_homog_matrix())
        rospy.loginfo("Transforming object pointcloud to map")
        pointcloud_transformed = geometry.transform_PointCloud2(pointcloud,
                                                                transform,
                                                                '/map')
        contour =  get_image_contour(camera_info, pointcloud_transformed)
        cv2.drawContours(rgb_image, [contour], 0, colours[j%len(colours)], 2)
        classification = "%s (%f)" % (c.identification.class_type[0], c.identification.class_type[1])
        print contour[0]
        cv2.putText(rgb_image,  classification, (contour[0][0][0], contour[0][0][1]),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, colours[j%len(colours)], 2)
        
#    cv2.imshow('image', rgb_image)
#    cv2.waitKey(30)
    return rgb_image
        
    
def get_image_contour(camera_info, pointcloud, pt_meld=1):
    pinhole = PinholeCameraModel()
    pinhole.fromCameraInfo(camera_info)
    img =  np.zeros((camera_info.height, camera_info.width, 1), np.uint8)
    for pt in pc2.read_points(pointcloud): # assume x,y,z
        u, v = pinhole.project3dToPixel(pt)
        cv2.circle(img, (int(u), int(v)), pt_meld, 255, -1)
        img[v, u] = 255
    contours, hierarchy = cv2.findContours(img,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    return contours[0]

def create_robblog(table_name,  timestamp):
    """ creates a robblog entry for observation at timestamp (or closest)"""
    msg_store_blog = MessageStoreProxy(collection='robblog')
    entry =  "# Table observation\ntable name: %s\n"
    e = RobblogEntry(title='Table Observation at ' + datetime.datetime.now().strftime("%I:%M%p"))
    e.body = 'I looked at ' + table_name + ' and this is what I found:\n\n'
    img =  create_table_observation_image(table_name, timestamp)
    bridge = CvBridge()
    ros_img = bridge.cv2_to_imgmsg(img)
    img_id = msg_store_blog.insert(ros_img)
    e.body += '![Image of the door](ObjectID(%s))' % img_id
    msg_store_blog.insert(e)
        

class PointCloudVisualiser(object):
    def __init__(self, topic="pointcloud_visualise"):
        self._pointclouds = []
        self._pub = rospy.Publisher(topic, PointCloud2)
    
    def add_cloud(self, cld):
        assert isinstance(cld, PointCloud2)
        if len(self._pointclouds) > 0:
            assert cld.point_step == self._pointclouds[0].point_step
            # TODO: dirty assumption of the fields being the same
            # TODO: dirty assumption of header being the same
        self._pointclouds.append(cld)
    
    def clear(self):
        self._pointclouds = []
        
    def publish(self):
        if len(self._pointclouds) < 1:
            return
        pts = []
        colours = [0xFF0000, 0x00FF00, 0x0000FF, 
                   0xFFFF00, 0xFF00FF, 0x00FFFF]
        for i, cloud in enumerate(self._pointclouds):
            rgb =  colours[i%len(colours)]
            pts.extend([ (p[0], p[1], p[2], rgb) for p in pc2.read_points(cloud,
                                                                 ['x', 'y', 'z']) ])
        fields = [PointField('x', 0, PointField.FLOAT32, 1),
                  PointField('y', 4, PointField.FLOAT32, 1),
                  PointField('z', 8, PointField.FLOAT32, 1), 
                  PointField('rgb', 12, PointField.UINT32, 1)]
        pointcloud = pc2.create_cloud(self._pointclouds[0].header,
                                      fields, pts)
        self._pub.publish(pointcloud)