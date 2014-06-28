#!/usr/bin/env python
import rospy
import json
 
from qsr_kb.srv import PrologQuery
from qsr_kb.srv import PrologQueryResponse

from swiplclient import PrologDB
from urllib2 import HTTPError

class KB(object):

    def __init__(self):

        rospy.init_node('kb_server')

        PL_HOST = rospy.get_param('~PL_HOST', 'localhost')
        PL_PORT = rospy.get_param('~PL_PORT', 5000)
        PL_PASSWORD = rospy.get_param('~PL_PASSWORD', 'xyzzy') 

        # Connect to the Prolog HTTP server
        self.kb = PrologDB(PL_HOST, PL_PORT, PL_PASSWORD)

        self.s = rospy.Service('kb', PrologQuery, self.handle_query)
        rospy.loginfo("KB up and running...")

        rospy.spin()

    def handle_query(self, req):
        rospy.loginfo("Query:  %s " % (req.query))
        try:
            solution = json.dumps(self.kb.query(req.query))
            rospy.loginfo("Solution:  %s " % (solution))
        except urllib2.HTTPError, e:
            rospy.logerr(e)
            return PrologQueryResponse() 
        return PrologQueryResponse(solution) 

if __name__ == "__main__":
    KB()
