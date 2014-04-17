#!/usr/bin/env python
import roslib; roslib.load_manifest("manual_table_storer")
import rospy
import sys

from strands_perception_msgs.msg import Table
from ros_datacentre.message_store import MessageStoreProxy
import tf
from geometry_msgs.msg import Point32, PoseStamped

# Beautiful hand coded table sizes in such a sensible place, great code. :-|
TABLES={}
TABLES["LabType1"]=[(0,0,0),
                    (1.795,0,0),
                    (1.795,0.84,0),
                    (0,0.84,0) ]
TABLES["LGType1"]=[(0,0,0),
                   (0,1,0),
                   (1,1,0),
                   (1,0.2,0)]
TABLES["LGType2"]=[(0,0,0),
                   (1.3,0,0),
                   (1.3,0.6,0),
                   (1.0,0.6,0),
                   (0.6,1.0,0),
                   (0.6,1.3,0),
                   (0,1.3,0)] 
TABLES["LGType3"]=[(0,0,0), (1.2,0,0),(1.2,0.65,0),(0.8,1.8,0),(0,1.8,0)]
TABLES["LGType4"]=[(0,0,0),
                   (0.4,0,0),
                   (0.4,0.6,0),
                   (-1.0,.6,0),
                   (-1,-1.0,0),
                   (-0.4,-1,0),
                   (-0.4,-0.6,0)]
TABLES["LGType1_"]=[(0,0,0),
                    (0, 0.8,0),
                    (1.0,0.8,0),
                    (1,-0.2,0)]

class TableInserter(object):
    def __init__(self, table_type,name):
        rospy.init_node("table_inserter", anonymous=True)

        self._msg_store=MessageStoreProxy()

        print ("Inserting a table of type "+table_type)
        if not TABLES.has_key(table_type):
            print ("ERROR: Unknown table type")
            sys.exit(1)

        self._pose=None
        self._tf_listener=tf.TransformListener()
        
        T=Table()
        T.table_id=name
        T.header.frame_id="/map"
        for p in TABLES[table_type]:
            pt=Point32()
            pt.x=p[0]
            pt.y=p[1]
            pt.z=p[2]
            T.tabletop.points.append(pt)

        # Wait for transformation to chessboard....
        rospy.loginfo("Waiting to see chessboard!")
        sub=rospy.Subscriber("/chessboard_pose", PoseStamped, self._chessboard_pose_cb)
        while self._pose is None:
            rospy.sleep(1)
        sub.unregister()

        
        rospy.loginfo("Got pose. Transforming to map frame...")
        self._pose = self._tf_listener.transformPose("/map", self._pose)
        
        rospy.loginfo("Storing in datacentre.")
        # Set table pose
        T.pose.pose = self._pose.pose
        
        # Store it
        self._msg_store.insert(T)
        
    def _chessboard_pose_cb(self,pose):
        self._pose=pose
        
if __name__=="__main__":
    if len(sys.argv) != 3:
        print "Wrong args; must give table type and table name"
        sys.exit(1)
    inserter = TableInserter(sys.argv[1], sys.argv[2])
