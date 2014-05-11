#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <sensor_msgs/PointCloud2.h>
#include <singleview_object_recognizer/CheckObjectPresenceAction.h>
#include <recognition_srv_definitions/recognize.h>
#include "ros_datacentre/message_store.h"
#include "ros_datacentre_msgs/StringPairList.h"
#include <std_msgs/Int32.h>
#include <scitos_ptu/PtuGotoAction.h>

using namespace std_msgs;
using namespace ros_datacentre;
using namespace ros_datacentre_msgs;
using namespace std;

class CheckObjectPresenceAction
{

protected:

  boost::shared_ptr<actionlib::SimpleActionServer<singleview_object_recognizer::CheckObjectPresenceAction> > as_;
  std::string action_name_;
  ros::NodeHandle nh_;

  singleview_object_recognizer::CheckObjectPresenceFeedback feedback_;
  singleview_object_recognizer::CheckObjectPresenceResult result_;
  ros::Subscriber sub_;
  sensor_msgs::PointCloud2ConstPtr cloud_;
  bool got_cloud_;
  std::string topic_;

public:
    
  CheckObjectPresenceAction(std::string name):
    //as_(nh_, name, boost::bind(&CheckObjectPresenceAction::executeCB, this, _1), false),
    action_name_(name)
  {
    as_.reset(new actionlib::SimpleActionServer<singleview_object_recognizer::CheckObjectPresenceAction>
            (nh_, name, boost::bind(&CheckObjectPresenceAction::executeCB, this, _1), false));
    //as_.registerGoalCallback(boost::bind(&CheckObjectPresenceAction::executeCB, this, _1));
    topic_ = "/head_xtion/depth_registered/points";
    as_->start();
    ROS_INFO("Action server started %s\n", name.c_str());
  }

  ~CheckObjectPresenceAction(void)
  {

  }

  void
  getCloud (const sensor_msgs::PointCloud2::ConstPtr& msg)
  {
      cloud_ = msg;
      got_cloud_ = true;
  }

  void movePTU(float pan, float tilt)
  {
      ROS_INFO("Moving PTU to %f %f", pan, tilt);
      actionlib::SimpleActionClient<scitos_ptu::PtuGotoAction> ptu("/SetPTUState", true);
      ptu.waitForServer();
      ROS_INFO("PTU server is active\n");
      scitos_ptu::PtuGotoGoal ptuGoal;
      ptuGoal.pan = pan;
      ptuGoal.tilt = tilt;
      ptuGoal.pan_vel = 20; // 20 is a reasonable default choice
      ptuGoal.tilt_vel = 20; // 20 is a reasonable default choice
      ptu.sendGoal(ptuGoal);
      bool finished_before_timeout = ptu.waitForResult(ros::Duration(30.0));
      if (!finished_before_timeout)
      {
        ROS_ERROR("Failed to move the PTU.");
        feedback_.status = "Unable to move PTU";
        result_.found = 0;
        as_->setAborted(result_);
      }
      else
      {
          ROS_DEBUG("Managed to move PTU\n");
      }
  }

  void executeCB(const singleview_object_recognizer::CheckObjectPresenceGoalConstPtr &goal)
  {
    // helper variables
    got_cloud_ = false;
    result_.found = 0;

    //move pan-tilt to goal view
    movePTU(goal->ptu_pan, goal->ptu_tilt);

    //get point cloud
    feedback_.status = "Getting cloud";
    ros::Subscriber sub_pc = nh_.subscribe (topic_, 1, &CheckObjectPresenceAction::getCloud, this);
    ros::Rate loop_rate (0.5);

    while (!got_cloud_ && ros::ok ())
    {
        if(as_->isPreemptRequested())
        {
            ROS_INFO("%s: Preempted", action_name_.c_str());
            as_->setPreempted();
            return;
        }

        ROS_INFO("Trying to get cloud from topic: %s\n", topic_.c_str());
        loop_rate.sleep ();
    }

    ROS_DEBUG("Got cloud, going to call service\n");

    //call recognition service
    feedback_.status = "Calling service";
    ros::ServiceClient client = nh_.serviceClient<recognition_srv_definitions::recognize>("mp_recognition");
    recognition_srv_definitions::recognize srv;
    srv.request.cloud = *cloud_;

    //parse service call
    std::string object_searched = goal->object_id;

    std::cout << "Looking for object with id:" << object_searched << std::endl;

    if (client.call(srv))
    {
        //parse result checking if the desired goal object is there or not, return true or false
        feedback_.status = "Parsing results";

        std::cout << "Object ids found:" << static_cast<int>(srv.response.ids.size()) << std::endl;

        for(size_t i=0; i < srv.response.ids.size(); i++)
        {
          std::cout << "   => " << srv.response.ids[i] << std::endl;
          std::string id = srv.response.ids[i].data;
          if(object_searched.compare(id) == 0)
          {
              result_.found = 1;
              break;
          }
        }
    }
    else
    {
        feedback_.status = "There was an error calling the service\n";
    }

    //move pan-tilt to (0,0)
    movePTU(0,0);

    feedback_.status = "Logging data";

    //log point cloud, result and object_id
    ros_datacentre::MessageStoreProxy messageStore(nh_, "checkObjectPresence");

    std::vector< std::pair<std::string, std::string> > stored;
    // now add objects and store ids with the addition of type strings for safety. The types are not necessary unless you want to do some kind of reflection on this data later.

    std_msgs::Int32 found_result;
    found_result.data = result_.found;

    std_msgs::String object_id_ros_msg;
    object_id_ros_msg.data = goal->object_id;

    stored.push_back( std::make_pair(get_ros_type(*cloud_), messageStore.insert(*cloud_)) );
    stored.push_back( std::make_pair(get_ros_type(found_result), messageStore.insert(found_result)) );
    stored.push_back( std::make_pair(get_ros_type(object_id_ros_msg), messageStore.insert(object_id_ros_msg)) );

    StringPairList spl;
    for(auto & pair : stored) {
        spl.pairs.push_back(ros_datacentre::makePair(pair.first, pair.second));
    }

    // and add some descriptive information
    mongo::BSONObjBuilder metaBuilder;
    metaBuilder.append("description", "checkObjectPresence result");
    metaBuilder.append("result_time", mongo::Date_t(ros::Time::now().toSec() * 1000));

    // and store
    messageStore.insert(spl, metaBuilder.obj());

    feedback_.status = "Succeeded";
    ROS_INFO("%s: Succeeded", action_name_.c_str());
    as_->setSucceeded(result_);
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "CheckObjectPresenceAction");

  CheckObjectPresenceAction object_presence(ros::this_node::getName());
  ros::spin();

  return 0;
}
