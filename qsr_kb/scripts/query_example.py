#!/usr/bin/env python

import sys
import rospy
import json
from qsr_kb.srv import *

def make_query(query):
    rospy.wait_for_service('qsrvis')
    try:
        swipl = rospy.ServiceProxy('qsrvis', PrologQuery)
        resp = swipl(query)
        return json.loads(resp.solution)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def create_event(relations, location, classifications):
    return "create_event(EVT," + str(relations) + "," + str(location) + "," + str(classifications) + "), new_vis." 

def usage():
    return "%s"%sys.argv[0]

if __name__ == "__main__":


    #################################
    # Asserting a new scene in the KB
    #################################
    
    # query =  """create_event(EVT,
    #                        [['left-of', 'keyboard', 'cup'], ['left-of', 'monitor', 'cup'],
    #                         ['behind', 'keyboard', 'cup'], ['in-front-of', 'monitor', 'cup'],
    #                         ['in-front-of', 'keyboard', 'monitor'], ['right-of', 'cup', 'monitor']],
    #                         'table27', 
    #                        [['BU', [['keyboard', 'Keyboard', 0.8], ['keyboard', 'Monitor', 0.2], 
    #                                 ['cup', 'Cup', 0.4], ['cup', 'Mouse', 0.5], ['cup', 'Keyboard', 0.1], 
    #                                 ['monitor', 'Keyboard', 0.1], ['monitor', 'Monitor', 0.9], 
    #                                 ['mouse', 'Cup', 0.9], ['mouse', 'Mouse', 0.1]]], 
    #                        ['TD', [['keyboard', 'Keyboard', 0.9], ['keyboard', 'Monitor', 0.1], 
    #                                 ['cup', 'Cup', 0.6], ['cup', 'Mouse', 0.2], ['cup', 'Keyboard', 0.2], 
    #                                 ['monitor', 'Keyboard', 0.1], ['monitor', 'Monitor', 0.9], 
    #                                 ['mouse', 'Cup', 0.1], ['mouse', 'Mouse', 0.9]]]]), new_vis.""" 


    
    rels = [['left-of', 'keyboard', 'cup'], ['left-of', 'monitor', 'cup'],
                            ['behind', 'keyboard', 'cup'], ['in-front-of', 'monitor', 'cup'],
                            ['in-front-of', 'keyboard', 'monitor'], ['right-of', 'cup', 'monitor']]

    loc = 'table27'

    cls = [['BU', [['keyboard', 'Keyboard', 0.8], ['keyboard', 'Monitor', 0.2], 
                                    ['cup', 'Cup', 0.4], ['cup', 'Mouse', 0.5], ['cup', 'Keyboard', 0.1], 
                                    ['monitor', 'Keyboard', 0.1], ['monitor', 'Monitor', 0.9], 
                                    ['mouse', 'Cup', 0.9], ['mouse', 'Mouse', 0.1]]], 
           ['TD', [['keyboard', 'Keyboard', 0.9], ['keyboard', 'Monitor', 0.1], 
                                    ['cup', 'Cup', 0.6], ['cup', 'Mouse', 0.2], ['cup', 'Keyboard', 0.2], 
                                    ['monitor', 'Keyboard', 0.1], ['monitor', 'Monitor', 0.9], 
                                    ['mouse', 'Cup', 0.1], ['mouse', 'Mouse', 0.9]]]]

    query = create_event(rels, loc, cls)
    
    print "Loading example: %s"%(query)
    print "Solution: %s"%(make_query(query))


    ##################################
    # Querying the last asserted scene
    ##################################

    
    # Get all relations in the scene. 
    query = "qsrT(R,O1,O2,Q), vis([Q])."

    print "Asking for relations: %s"%(query)
    print "Solution: %s"%(make_query(query))

    query = "next_vis(X)."
    sol = make_query(query)
    while sol:
        key = raw_input("Press Enter to visualise next solution...")
        sol = make_query(query)

    

    


    
