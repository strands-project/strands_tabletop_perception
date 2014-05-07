import rospy
import copy
import numpy as np

from mongo import MongoDocument, MongoTransformable, MongoConnection
from geometry import Pose
from identification import ObjectIdentification
from observation import Observation,  MessageStoreObject
from exceptions import StateException

from ros_datacentre.message_store import MessageStoreProxy

class Object(MongoDocument):
    def __init__(self, mongo=None):
        super(Object, self).__init__()
        if mongo is not None:
            # this will create the document live..
            self._connect(mongo) 
        self._children = []
        self._parent = None
        self._bounding_box = None #BBoxArray(bbox)
        self._observations = None
        
        self._life_start = rospy.Time.now().to_time()
        self._life_end = None
        
        self.identifications = {}
        self.identification = ObjectIdentification()
        
        self._msg_store_objects =  [] # a list of object IDs (strings) 

        self._observations =  [] # a list of observation objects
        
        self._poses = [] 
        
    # TODO: properies for all, remove MongoDocument?
    
    @property
    def pose(self):
        if len(self._poses) < 1:
            raise StateException("NOPOSE")
        return copy.deepcopy(self._poses[-1])
    
    @property
    def position(self):
        if len(self._poses) < 1:
            raise StateException("NOPOSE")
        return copy.deepcopy(self._poses[-1]['position'])
    
    @property
    def quaternion(self):
        if len(self._poses) < 1:
            raise StateException("NOPOSE")
        return copy.deepcopy(self._poses[-1]['quaternion'])
    
    @property
    def pose_homog_transform(self):
        if len(self._poses) < 1:
            raise StateException("NOPOSE")
        return self._poses[-1].as_homog_matrix
        
    def cut(self):
        self._life_end = rospy.Time.now().to_time()
    
    def cut_all_children(self):
        for i in self._children:
            World.get_object(i).cut()
    
    @property
    def name(self):
        return self.key
    
    @name.setter
    def name(self, name):
        # TODO: check doesn't already exist
        self.key = name
    

    def add_identification(self, classifier_id, identification):
        if not self.identifications.has_key(classifier_id):
            self.identifications[classifier_id] = []
        # TODO check time consistency
        self.identifications[classifier_id].append(identification)
        self.identifications = self.identifications #force mongo update
        # TODO: don't duplicate? include classifier_id?
        self.identification = identification
        
    def add_pose(self, pose):
        # TODO check time consistency
        p = copy.deepcopy(pose)
        self._poses.append(p) #[str(p)] = p
        self._poses = self._poses #force mongo update
        
    def add_observation(self, observation):
        assert isinstance(observation,  Observation)
        self._observations.append(observation)
        self._observations =  self._observations
        
    def add_msg_store(self, message):
        assert isinstance(message, MessageStoreObject)
        self._msg_store_objects.append(message)
        self._msg_store_objects = self._msg_store_objects
        
    def get_identification(self, classifier_id=None):
        if classifier_id is None:
            return self.identification
        return self.identifications[classifier_id][-1]
    
    def get_children_names(self):
        return copy.copy(self._children)
    
    def add_child(self, child_object):
        #self._children.append(child_object.get_name)
        # have to recreate to catch in setattr
        self._children+=[child_object.name]

    def get_message_store_messages(self, typ=None):
        msgs = []
        proxy = MessageStoreProxy()
        for msg in self._msg_store_objects:
            if typ != msg.typ and typ is not None:
                continue
            proxy.database =  msg.database
            proxy.collection =  msg.collection
            msgs.append(proxy.query_id(msg.obj_id, msg.typ))
        return msgs
        
    @classmethod
    def _mongo_encode(cls, class_object):
        doc = {}
        doc.update(class_object.__dict__)
        try:
            doc.pop("_MongoDocument__mongo")
            doc.pop("_MongoDocument__connected")
        except KeyError:
            print "Warning: no no no"
        doc["__pyobject_class_type"] = class_object.get_pyoboject_class_string()
        doc = copy.deepcopy(doc)
        return doc
    
        
class World(object):
    def __init__(self, database_name='world_state', server_host=None,
                 server_port=None):
        self._mongo = MongoConnection(database_name, server_host, server_port)

    def does_object_exist(self, object_name):
        result = self._mongo.database.Objects.find(
            {"__pyobject_class_type": Object.get_pyoboject_class_string(),
             'key': object_name,})

        return result.count() == 1

    
    def get_object(self, object_name):  
        result = self._mongo.database.Objects.find(
            {"__pyobject_class_type": Object.get_pyoboject_class_string(),
             'key': object_name,})
        
        if result.count() != 1:
            raise Exception("get_object failed to find object '%s' in database."%object_name)
        found = result[0]
        found._connect(self._mongo)

        return found
    
    def create_object(self, object_name=None):
        new_object = Object()
        if object_name is not None:
            new_object.set_name(object_name)
        new_id = self._mongo.database.Objects.insert(Object._mongo_encode(new_object))
        new_object._id = new_id
        new_object._connect(self._mongo)
        return new_object
    
    def remove_object(self, obj):
        if not isinstance(obj, Object):
            obj = self.get_object(obj)
        new_id = self._mongo.database.ObjectWasteland.save(Object._mongo_encode(obj))
        self._mongo.database.Objects.remove({
            "__pyobject_class_type": Object.get_pyoboject_class_string(),
             'key': obj.name, })


    def get_objects_of_type(self, ob_type, min_confidence=None):
        if min_confidence is None:
            result = self._mongo.database.Objects.find(
                {"__pyobject_class_type": Object.get_pyoboject_class_string(),
                 'identification.class_type': ob_type,})
        else:
            raise NotImplementedError("TODO: implement confidence check.")
        
        objs = []
        for r in result:
            r._connect(self._mongo)
            objs.append(r)
        return objs
    
    def get_root_objects(self):
        """
        return all objects that have no parent
        """
        result = self._mongo.database.Objects.find(
                {"__pyobject_class_type": Object.get_pyoboject_class_string(),
                 '_parent': None,})
        objs = []
        for r in result:
            r._connect(self._mongo)
            objs.append(r)
        return objs