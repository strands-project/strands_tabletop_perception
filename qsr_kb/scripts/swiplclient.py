import urllib2
import json

class PrologDB(object):
    def __init__(self, host, port, password):
        connection_details = (host, port)
        self.url = "http://%s:%d" % connection_details
        self.query_url = "%s/query" % self.url
        self.add_url = "%s/add" % self.url
        self.remove_url = "%s/remove" % self.url
        self.password = password
        self.headers = { 'Content-Type' : 'application/json' }
    def __construct_json(self, action):
        # It's necessary to use construct_json() to construct a json dictionary 
        #   rather than constructing a Python dictionary and using json.dumps().  
        #   This is because Prolog can only efficiently read the dictionary 
        #   keys in a pre-defined order, and Python dictionaries do not 
        #   preserve key order.
        password = self.password
        action_part = '"'.join(['action',':',action])
        password_part = '"'.join(['password',':',password])
        jdata = '{ "%s", "%s" }' % (action_part, password_part)
        return jdata
    def __server_call(self, url, pl_string):
        jdata = self.__construct_json(pl_string)
        req = urllib2.Request(url, jdata, self.headers)
        return json.loads(urllib2.urlopen(req).read())
    def add(self, rule_string):
        return self.__server_call(self.add_url, rule_string)
    def remove(self, rule_string):
        return self.__server_call(self.remove_url, rule_string)
    def query(self, query_string):
        return self.__server_call(self.query_url, query_string)
        
if __name__ == "__main__":
    # Example:
    
    PL_HOST = "localhost"
    PL_PORT = 5000
    PL_PASSWORD = "xyzzy"
    
    prolog = PrologDB(PL_HOST, PL_PORT, PL_PASSWORD)
    
    # Sample assertions and query:
    
    prolog.add("father(jack, john)")
    prolog.add("brothers(joey, jack)")
    prolog.add("brothers(A,B) :- "
                            "B @< A, brothers(B,A)") # Make brothers/2 symmetric
    prolog.add("uncle(X,Y) :- brothers(Z,X), father(Z,Y)")
    print prolog.query("uncle(Uncle,Nephew)")
    
    
    
