import rospy
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2


from state import World, Object
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
        print "-> ", o.name, " ==> ", o.identification.class_type, " ", o.get_children_names()
        for o2 in [w.get_object(ob) for ob in o.get_children_names()]:
            print "-> -> ", o2.name, " ==> ", o2.identification.class_type
    pass


class PointcloudVisualiser(object):
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