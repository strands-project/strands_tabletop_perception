#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <sensor_msgs/PointCloud2.h>
#include <singleview_object_recognizer/CheckObjectPresenceAction.h>
#include <recognition_srv_definitions/recognize.h>
#include <scitos_ptu/PtuGotoAction.h>

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
    topic_ = "/camera/depth_registered/points";
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

  void executeCB(const singleview_object_recognizer::CheckObjectPresenceGoalConstPtr &goal)
  {
    // helper variables
    got_cloud_ = false;
    result_.found = 0;

    //move pan-tilt to goal view
    ROS_INFO("Moving PTU to %f %f", goal->ptu_pan, goal->ptu_tilt);
    actionlib::SimpleActionClient<scitos_ptu::PtuGotoAction> ptu("/PtuGotoAction", true);
    ptu.waitForServer();
    scitos_ptu::PtuGotoGoal ptuGoal;
    ptuGoal.pan = goal->ptu_pan;
    ptuGoal.tilt = goal->ptu_tilt;
    ptuGoal.pan_vel = 20; // 20 is a reasonable default choice
    ptuGoal.tilt = 20; // 20 is a reasonable default choice
    ptu.sendGoal(ptuGoal);
    bool finished_before_timeout = ptu.waitForResult(ros::Duration(30.0));
    if (!finished_before_timeout)
      ROS_ERROR("Failed to move the PTU.");
    // TODO: now our own action should actually terminate with failure

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
