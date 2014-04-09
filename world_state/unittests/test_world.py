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
        
        w.remove_object(obj)
         
            
if __name__ == '__main__':
    import rosunit
    PKG='world_state'
    rosunit.unitrun(PKG, 'test_objects', TestObjects)
    