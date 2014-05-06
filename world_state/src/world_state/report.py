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

