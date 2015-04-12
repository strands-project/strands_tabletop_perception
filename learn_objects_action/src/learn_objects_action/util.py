import rospy 

def get_ros_service(srv_name, srv_type):
    try:
        rospy.loginfo("Initialising LeanObjects SM: "
                      "Getting '%s' service.."%srv_name)
        rospy.wait_for_service(srv_name, 5)
        rospy.loginfo("-> ok.")
    except:
        rospy.logerr( "Service '%s' not available. Aborting"%srv_name)
        raise Exception("LeranObject SM can't initialise; missing service.")
    return rospy.ServiceProxy(srv_name, srv_type)
