#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <sensor_msgs/PointCloud2.h>
#include "ros_datacentre/message_store.h"
#include "ros_datacentre_msgs/StringPairList.h"
#include <std_msgs/Int32.h>
#include <scitos_ptu/PtuGotoAction.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions.h>
#include "geometry_msgs/Pose.h"
#include <scitos_apps_msgs/action_buttons.h>
#include "tf/transform_listener.h"
#include <tf/tf.h>
#include <scene_recorder/TakeSnapshotAction.h>
#include <tf2_msgs/TFMessage.h>

using namespace geometry_msgs;
using namespace ros_datacentre;
using namespace ros_datacentre_msgs;
using namespace std;

class SceneSnapshot
{
private:
    std::string camera_frame_;
    std::string base_frame_;
    geometry_msgs::Pose robot_pose_;
    boost::shared_ptr <pcl::PointCloud<pcl::PointXYZRGB>> pCloud_;
    bool cam_snapshot_done_;
    bool robot_pose_snapshot_done_;
    ros::Subscriber sub_tf_;
    ros::Subscriber sub_pc_;
    ros::NodeHandle nh_;
    boost::shared_ptr<actionlib::SimpleActionServer<scene_recorder::TakeSnapshotAction> > as_;
    boost::shared_ptr<MessageStoreProxy> messageStore;
    scene_recorder::TakeSnapshotActionFeedback feedback_;
    scene_recorder::TakeSnapshotActionResult result_;

public:
    SceneSnapshot(std::string name)
    {
        camera_frame_ = "/camera_depth_optical_frame";
        base_frame_ = "/camera_link";
        pCloud_.reset(new pcl::PointCloud<pcl::PointXYZRGB>() );
        cam_snapshot_done_ = false;
        robot_pose_snapshot_done_ = false;
        messageStore.reset(new MessageStoreProxy(nh_, "ws_observations"));

//        std::vector< boost::shared_ptr<ros_datacentre_msgs::SerialisedMessage> > snapshots;
//        messageStore->queryID<ros_datacentre_msgs::SerialisedMessage>("53be61345d90780f4b1252bf", snapshots);
//        for(size_t i=0; i<snapshots.size(); i++)
//        {
//            ros_datacentre_msgs::SerialisedMessage snapshot = *snapshots[i];
//        }

        as_.reset(new actionlib::SimpleActionServer<scene_recorder::TakeSnapshotAction>
                (nh_, name, boost::bind(&SceneSnapshot::actionCallback, this, _1), false));
        as_->start();
    }

    void robot_pose_callback(const geometry_msgs::Pose& msg)
    {
        ROS_INFO("I have received the robot pose.");
        robot_pose_ = msg;
        robot_pose_snapshot_done_ = true;
    }

    void actionCallback(const scene_recorder::TakeSnapshotGoalConstPtr &goal)
    {
        ROS_INFO("Got an action request.");
        getBaseCameraTransform(transform_);
        sub_tf_ = nh_.subscribe ("/robot_pose", 1, &SceneSnapshot::robot_pose_callback, this);
        sub_pc_ = nh_.subscribe ("/camera/depth_registered/points", 1, &SceneSnapshot::kinectCallback, this);

        while(!cam_snapshot_done_ )//|| !robot_pose_snapshot_done_)
        {
           std::cout << ".";
        }
        sub_pc_.shutdown();
        sub_tf_.shutdown();
        cam_snapshot_done_ = false;
        robot_pose_snapshot_done_ = false;
        std::string name("camera transform test2");
        //Insert something with a name, storing id too
        std::string id(messageStore->insertNamed(name, robot_pose_));
        geometry_msgs::Pose pose;

        //cast tf_transform to eigen::Matrix4f
        tf::Vector3 p = tf_transform_.getOrigin ();
        tf::Quaternion q = tf_transform_.getRotation ();

        pose.position.x = p.getX ();
        pose.position.y = p.getY ();
        pose.position.z = p.getZ ();

        pose.orientation.w = q.getW();
        pose.orientation.x = q.getX();
        pose.orientation.y = q.getY();
        pose.orientation.z = q.getZ();

        std::string id2(messageStore->insertNamed(name, pose));

        sensor_msgs::PointCloud2 rosCloud;
        pcl::toROSMsg(*pCloud_, rosCloud);
        std::string id3(messageStore->insertNamed(name, rosCloud));
        std::cout<<"Transform \""<<name<<"\" inserted with id "<< id << " and " << id2 << " and " << id3 << std::endl;
    }

    void kinectCallback ( const sensor_msgs::PointCloud2& msg )
    {
        ROS_INFO("I have received a new snapshot of the scene.");
        pcl::fromROSMsg(msg, *pCloud_);
        cam_snapshot_done_ = true;
    }

    bool
    getBaseCameraTransform (Eigen::Matrix<float, 4, 4, Eigen::RowMajor> & trans)
    {
        tf::TransformListener tf_listener;
        tf::StampedTransform tf_transform_;
        Eigen::Matrix<float, 4, 4, Eigen::RowMajor> transform_;

        std::cout << "Getting transform from:" << camera_frame_ << " to " << base_frame_ << std::endl;

        try
        {
            tf_listener.waitForTransform (base_frame_, camera_frame_, ros::Time (0), ros::Duration (5.0));
            tf_listener.lookupTransform (base_frame_, camera_frame_, ros::Time (0), tf_transform_);
        }
        catch (tf::TransformException ex)
        {
            std::cout << ex.what () << std::endl;
            return false;
        }
        //cast tf_transform to eigen::Matrix4f
        tf::Vector3 p = tf_transform_.getOrigin ();
        tf::Quaternion q = tf_transform_.getRotation ();


        Eigen::Vector3f translation = Eigen::Vector3f (p.getX (), p.getY (), p.getZ ());

        Eigen::Quaternion<float> rot (q.getW (), q.getX (), q.getY (), q.getZ ());
        rot.normalize();
        Eigen::Matrix3f rotation = rot.toRotationMatrix();

        trans.setIdentity();
        trans.block<3,1>(0,3) = translation;
        trans.block<3,3>(0,0) = rotation;
        return true;
    }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "TakeSnapshot");
  SceneSnapshot robot_scene(ros::this_node::getName());
  ros::spin();
  return 0;
}
