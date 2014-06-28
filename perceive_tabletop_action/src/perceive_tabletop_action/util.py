from world_state.state import World
import rospy

def get_table(table_name):
    world = World()
    if not world.does_object_exist(table_name):
        rospy.logerr("Object (%s) does not exist"%table_name)
        return None
    
    table =  world.get_object(table_name)
    if table.identification.class_type[0] != "Table":
        rospy.logerr("Object (%s) is not a table!"%userdata.table_id)
        return None

    return table
    
def get_node_for_table(table_name):
    """
    Given a table name, returns a topological map node name
    """
    table = get_table(table_name)
    if table is None:
        return 
    return table._msg_store_objects[0].retrieve().topological_node
