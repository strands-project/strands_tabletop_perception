#!/usr/bin/env python

import rospy
import ros_datacentre_msgs.srv as dc_srv
from ros_datacentre_msgs.msg import StringPair
import ros_datacentre.util as dc_util
from ros_datacentre.message_store import MessageStoreProxy

import StringIO

from strands_executive_msgs import task_utils
from strands_executive_msgs.msg import Task
from strands_executive_msgs.srv import AddTask, SetExecutionStatus



if __name__ == '__main__':
    rospy.init_node("perceive_table_task_client")

    # need message store to pass objects around
    msg_store = MessageStoreProxy() 

    try:

	task = Task(start_node_id='WayPoint4', action='perceive_tabletop')
        task_utils.add_string_argument(task, 'test_lg_1')

        print task

        # now register this with the executor
        add_task_srv_name = '/task_executor/add_task'
        set_exe_stat_srv_name = '/task_executor/set_execution_status'
        rospy.loginfo("Waiting for task_executor service...")
        rospy.wait_for_service(add_task_srv_name)
        rospy.wait_for_service(set_exe_stat_srv_name)
        rospy.loginfo("Done")        
        
        add_task_srv = rospy.ServiceProxy(add_task_srv_name, AddTask)
        set_execution_status = rospy.ServiceProxy(set_exe_stat_srv_name, SetExecutionStatus)
        print add_task_srv(task)

        # Make sure the task executor is running
 #       set_execution_status(True)


    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


        


