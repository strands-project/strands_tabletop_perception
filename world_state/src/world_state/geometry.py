import rospy
from mongo import MongoTransformable
import numpy as np
from geometry_msgs.msg import PoseStamped, Pose, PoseWithCovariance

class Point(MongoTransformable):
    def __init__(self, x=0, y=0, z=0):
        self.x = x
        self.y = y
        self.z = z
            
class Quaternion(MongoTransformable):
    def __init__(self, x=0, y=0, z=0, w=1):
        self.x = x
        self.y = y
        self.z = z
        self.w = w
    
    @property
    def as_numpy(self):
        return np.array([self.x, self.y, self.z, self.w])
        
class Pose(MongoTransformable):
    def __init__(self, position=None, quaternion=None):
        self.stamp = rospy.Time.now().to_time()
        self.ros_frame_id = ""
        if position is not None:
            self.position = copy.deepcopy(position)
        else:
            self.position = Point()
            
        if quaternion is not None:
            self.quaternion = copy.deepcopy(quaternion)
        else:
            self.quaternion = Quaternion()
        
    @classmethod
    def create_zero(cls, timestamp=None):
        p = cls() #Point(0, 0, 0), Quaternion(0, 0, 0, 1))
        if timestamp is not None:
            p.stamp = timestamp
        return p
    
    @classmethod
    def from_ros_msg(cls, ros_msg):
        #assert (isinstance(ros_msg, Pose) or isinstance(ros_msg, PoseStamped)
                #or isinstance(ros_msg, PoseWithCovariance))
        pose = ros_msg
        p = cls()
        if hasattr(pose, "pose"):
            p.stamp = pose.header.stamp.to_time()
            p.ros_frame_id = pose.header.ros_frame_id
            pose = pose.pose
        p.position.x = pose.position.x
        p.position.y = pose.position.y
        p.position.z = pose.position.z
        p.quaternion.x = pose.orientation.x
        p.quaternion.y = pose.orientation.y
        p.quaternion.z = pose.orientation.z
        p.quaternion.w = pose.orientation.w
        return p
    
    def __str__(self):
        return "{0:d}".format(int(self.stamp*1000))
    
    @property
    def as_homog_matrix(self):
        """Return homogeneous transformation matrix for this pose.
        """
        # Borrowed from rospy tf.transformations code
        q = self.quaternion.as_numpy
        nq = np.dot(q, q)
        if nq < np.spacing(0):
            return np.identity(4)
        q *= math.sqrt(2.0 / nq)
        q = np.outer(q, q)
        return np.array((
            (1.0-q[1, 1]-q[2, 2], q[0, 1]-q[2, 3], q[0, 2]+q[1, 3],  self.postion.x),
            ( q[0, 1]+q[2, 3], 1.0-q[0, 0]-q[2, 2], q[1, 2]-q[0, 3], self.postion.y),
            ( q[0, 2]-q[1, 3], q[1, 2]+q[0, 3], 1.0-q[0, 0]-q[1, 1], self.postion.z),
            ( 0.0, 0.0, 0.0, 1.0)
            ), dtype=np.float64)

            


class BBoxArray(MongoTransformable):
    """ Bounding box of an object
    """
    def __init__(self, bbox):
        self.points = bbox
        # Calc x_min and x_max for obj1
        x_sorted = sorted(bbox, key=itemgetter(0))
        self._x_min = x_sorted[0][0]
        self._x_max = x_sorted[7][0]

        # Calc y_min and y_max for obj
        y_sorted = sorted(bbox, key=itemgetter(1))
        self._y_min = y_sorted[0][1]
        self._y_max = y_sorted[7][1]

        # Calc z_min and z_max for obj
        z_sorted = sorted(bbox, key=itemgetter(2))
        self._z_min = z_sorted[0][2]
        self._z_max = z_sorted[7][2]
        
    @property
    def x_min(self):
        return self.x_min

    @property
    def x_max(self):
        return self.x_max

    @property
    def y_min(self):
        return self.y_min

    @property
    def y_max(self):
        return self.y_max
    
    @property
    def z_min(self):
        return self.z_min

    @property
    def z_max(self):
        return self.z_max
    
    @property
    def size(self):
        return (self.x_max - self.x_min,
                self.y_max - self.y_min,
                self.z_max - self.z_min )
    
    @property
    def volume(self):
        return reduce(lambda x, y: x*y, self.size)
                

