/*
 * shape_simple_classifier_node.cpp
 *
 *  Created on: Sep 7, 2013
 *      Author: aitor
 */

#define EIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET

#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "std_msgs/Float32.h"

#include <pcl/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions.h>
#include <pcl/common/angles.h>
#include <pcl/common/transforms.h>
#include <pcl/common/time.h>

#include "camera_srv_definitions/start_tracker.h"
#include "camera_srv_definitions/stop_tracker.h"
#include "camera_srv_definitions/visualize_compound.h"

#include "v4r/KeypointConversions/convertImage.hpp"
#include "v4r/KeypointConversions/convertCloud.hpp"
#include "v4r/KeypointTools/invPose.hpp"
#include "v4r/KeypointTools/toString.hpp"
#include "v4r/KeypointTools/PoseIO.hpp"
#include "v4r/KeypointSlam/KeypointSlamRGBD2.hh"
#include "v4r/KeypointSlam/ProjBundleAdjuster.hh"
#include "v4r/KeypointTools/PointTypes.hpp"
#include "v4r/KeypointTools/ScopeTime.hpp"

class CamTracker
{
private:
  typedef pcl::PointXYZRGB PointT;
  boost::shared_ptr<ros::NodeHandle> n_;
  ros::ServiceServer cam_tracker_start_;
  ros::ServiceServer cam_tracker_stop_;
  ros::ServiceServer cam_tracker_vis_compound_;
  ros::Subscriber camera_topic_subscriber_;
  ros::Publisher confidence_publisher_;

  kp::KeypointSlamRGBD2::Parameter param;
  kp::KeypointSlamRGBD2::Ptr camtracker;

  double cos_min_delta_angle_;
  double sqr_min_cam_distance_;
  std::vector<Eigen::Matrix4f> cameras_;
  std::vector<std::pair<int, pcl::PointCloud<pcl::PointXYZRGB>::Ptr> > keyframes_;
  int num_clouds_;

  void drawConfidenceBar(cv::Mat &im, const double &conf)
  {
    int bar_start = 50, bar_end = 200;
    int diff = bar_end-bar_start;
    int draw_end = diff*conf;
    double col_scale = 255./(double)diff;
    cv::Point2f pt1(0,30);
    cv::Point2f pt2(0,30);
    cv::Vec3b col(0,0,0);

    if (draw_end<=0) draw_end = 1;

    for (int i=0; i<draw_end; i++)
    {
      col = cv::Vec3b(255-(i*col_scale), i*col_scale, 0);
      pt1.x = bar_start+i;
      pt2.x = bar_start+i+1;
      cv::line(im, pt1, pt2, CV_RGB(col[0],col[1],col[2]), 8);
    }
  }

  int selectFrames(const pcl::PointCloud<pcl::PointXYZRGB> &cloud,
                    int cam_id, const Eigen::Matrix4f &pose)
  {
    int type = 0;


    if (cam_id>=0)
    {
      type = 1;

      Eigen::Matrix4f inv_pose;
      kp::invPose(pose, inv_pose);

      unsigned z;
      for (z=0; z<cameras_.size(); z++)
      {
        if ( (inv_pose.block<3,1>(0,2).dot(cameras_[z].block<3,1>(0,2)) > cos_min_delta_angle_) &&
             (inv_pose.block<3,1>(0,3)-cameras_[z].block<3,1>(0,3)).squaredNorm() < sqr_min_cam_distance_ )
        {
          break;
        }
      }

      if (z>=cameras_.size())
      {
        type = 2;
        cameras_.push_back(inv_pose);
        keyframes_.push_back(std::make_pair(cam_id, pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>())));
        pcl::copyPointCloud(cloud, *(keyframes_.back().second));
        ROS_INFO("Added new keyframe**********************************************************");
      }
    }


    return type;
  }

  void trackNewCloud(const sensor_msgs::PointCloud2Ptr& msg)
  {

      pcl::ScopeTime t("trackNewCloud");

      if(num_clouds_ < 60)
      {
          num_clouds_++;
          return;
      }

      pcl::PointCloud<PointT>::Ptr scene (new pcl::PointCloud<PointT>);
      pcl::moveFromROSMsg (*msg, *scene);

      kp::DataMatrix2D<Eigen::Vector3f> kp_cloud;
      cv::Mat_<cv::Vec3b> image;

      kp::convertCloud(*scene, kp_cloud, image);

      double conf=0;
      int cam_idx=-1;
      Eigen::Matrix4f pose;

      bool is_ok = camtracker->track(image, kp_cloud, pose, conf, cam_idx);

      drawConfidenceBar(image, conf);
      cv::imshow("image", image);
      cv::waitKey(1);

      if(is_ok)
      {
          selectFrames(*scene, cam_idx, pose);
      }

      /*std_msgs::Float32 conf_mesage;
      conf_mesage.data = conf;
      confidence_publisher_.publish(conf_mesage);*/
  }

  void getCloud(const sensor_msgs::PointCloud2Ptr& msg)
  {
        trackNewCloud(msg);
  }

  bool
  start (camera_srv_definitions::start_tracker::Request & req,
         camera_srv_definitions::start_tracker::Response & response)
  {

        cameras_.clear();
        keyframes_.clear();
        num_clouds_ = 0;

        camera_topic_subscriber_ = n_->subscribe("/camera/depth_registered/points", 1, &CamTracker::getCloud, this);

        cv::Mat_<double> distCoeffs = cv::Mat::zeros(4, 1, CV_64F);
        cv::Mat_<double> intrinsic = cv::Mat_<double>::eye(3,3);
        intrinsic(0,0)=intrinsic(1,1)=525;
        intrinsic(0,2)=320, intrinsic(1,2)=240;

        camtracker.reset( new kp::KeypointSlamRGBD2(param) );
        camtracker->setCameraParameter(intrinsic,distCoeffs);


        confidence_publisher_ = n_->advertise<std_msgs::Float32>("cam_tracker_confidence", 1);

        return true;
  }

  bool
  stop (camera_srv_definitions::start_tracker::Request & req,
         camera_srv_definitions::start_tracker::Response & response)
  {
        camera_topic_subscriber_.shutdown();
        return true;
  }

  bool
  visCompound (camera_srv_definitions::visualize_compound::Request & req,
               camera_srv_definitions::visualize_compound::Response & response)
  {

        std::cout << "!Number of keyframes: " << cameras_.size() << std::endl;
        pcl::visualization::PCLVisualizer vis("compound cloud");
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr compound(new pcl::PointCloud<pcl::PointXYZRGB>);

        for(size_t i=0; i < cameras_.size(); i++)
        {
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr c(new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::transformPointCloud( *(keyframes_[i].second), *c, cameras_[i]);
            *compound += *c;
        }

        vis.addPointCloud(compound);
        vis.spin();

        return true;
  }

public:
  CamTracker ()
  {
    cos_min_delta_angle_ = cos(20*M_PI/180.);
    sqr_min_cam_distance_ = 1.*1.;
    num_clouds_ = 0;

    param.kd_param.rt_param.inl_dist = 0.01; //e.g. 0.01 .. table top, 0.03 ..rooms
    param.lk_param.rt_param.inl_dist = 0.01; //e.g. 0.01 .. table top, 0.03 ..rooms
    param.kt_param.rt_param.inl_dist = 0.03;  //e.g. 0.04 .. table top, 0.1 ..room
    param.om_param.kd_param.rt_param.inl_dist = 0.01; //e.g. 0.01 .. table top, 0.03 ..rooms
    param.om_param.kt_param.rt_param.inl_dist = 0.03;  //e.g. 0.04 .. table top, 0.1 ..room

  }

  void
  initialize (int argc, char ** argv)
  {

    n_.reset( new ros::NodeHandle ( "~" ) );

    cam_tracker_start_  = n_->advertiseService ("start_recording", &CamTracker::start, this);
    cam_tracker_stop_  = n_->advertiseService ("stop_recording", &CamTracker::stop, this);
    cam_tracker_vis_compound_  = n_->advertiseService ("vis_compound", &CamTracker::visCompound, this);

    ros::spin ();
  }
};

int
main (int argc, char ** argv)
{
  ros::init (argc, argv, "camera_tracker");

  CamTracker m;
  m.initialize (argc, argv);

  return 0;
}
