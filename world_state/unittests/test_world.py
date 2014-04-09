#!/usr/bin/env python

import sys
import unittest
from world_state.state import World, Object
from world_state.identification import ObjectIdentifcation

class TestWorld(unittest.TestCase):

    def test_world_add_object(self):
        w = World("world_state")
        obj = w.create_object()

        name = obj.name
        obj = w.get_object(obj.name)
        obj.add_identification("TableDetection",
                               ObjectIdentifcation({'Table': 0.2,
                                                    'Football': 0.3}))
        
        
        obj = w.get_object(obj.name)

        obj.alpha = 45

        self.assertEqual(name, obj.name)

        self.assertEqual(obj.get_identification("TableDetection").class_type,
                         ["Football", 0.3])
        
        #w.remove_object(obj)
import rospy
import nav_msgs
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
  
class Robot:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.th = 0
         
        self.pub = rospy.Publisher("/cmd_vel", Twist, latch=True)
        rospy.Subscriber("/odom",  Odometry,  self.odometry_callback)
         
         
    def odometry_callback(self,  odometry):
        quaternion = odometry.pose.pose.orientation
         
        q0 = quaternion.x
        q1 = quaternion.y
        q2 = quaternion.z
        q3 = quaternion.w
         
        self.th = math.atan2(2.*(q0*q1 + q2*q3), 1. - 2.*(q1**2 + q2**2))
        #self.th = 2*math.sin(q3/q2)
        self.x = odometry.pose.pose.position.x
        self.y = odometry.pose.pose.position.y
         
        #print (self.x, self.y, self.th)
     
    def set_vel(self,  v,  w):
        msg = Twist()
        msg.linear.x = v
        msg.angular.z = w       
        self.pub.publish(msg)         
            
if __name__ == '__main__':
    import rosunit
    PKG='world_state'
    rosunit.unitrun(PKG, 'test_objects', TestObjects)
    