import mongo

class Observation(object):
    # Snap shot of robot sensors & actuators
    pass

class MessageStoreObject(mongo.MongoTransformable):
    def __init__(self,  database, collection, obj_id, typ):
        self.database = database
        self.collection = collection
        self.obj_id = obj_id
        self.typ = typ