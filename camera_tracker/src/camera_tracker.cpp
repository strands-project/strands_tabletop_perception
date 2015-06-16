/*
 * shape_simple_classifier_node.cpp
 *
 *  Created on: Sep 7, 2013
 *      Author: aitor
 */

#define EIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET

#include <thread>
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/CameraInfo.h"
#include "std_msgs/Float32.h"
#include <tf/transform_broadcaster.h>

#include <pcl/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions.h>
#include <pcl/common/angles.h>
#include <pcl/common/transforms.h>
#include <pcl/common/time.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d_omp.h>

#include "camera_srv_definitions/start_tracker.h"
#include "camera_srv_definitions/stop_tracker.h"
#include "camera_srv_definitions/visualize_compound.h"
#include "camera_srv_definitions/get_tracking_results.h"
#include "camera_srv_definitions/do_ba.h"
#include "camera_srv_definitions/cleanup.h"

#include "v4r/KeypointConversions/convertImage.hpp"
#include "v4r/KeypointConversions/convertCloud.hpp"
#include "v4r/KeypointTools/invPose.hpp"
#include "v4r/KeypointTools/toString.hpp"
#include "v4r/KeypointTools/PoseIO.hpp"
#include "v4r/KeypointSlam/KeypointSlamRGBD2.hh"
#include "v4r/KeypointSlam/ProjBundleAdjuster.hh"
#include "v4r/KeypointTools/PointTypes.hpp"
#include "v4r/KeypointTools/ScopeTime.hpp"
#include <v4r/ORUtils/noise_model_based_cloud_integration.h>
#include <v4r/ORUtils/noise_models.h>

void saveToDisk(pcl::PointCloud<pcl::PointXYZRGB> scene,
                int saved_cloud)
{

    pcl::ScopeTime t("saving took....................");
    std::stringstream name;
    name << "/media/aitor14/DATA/camtracker/output_" << std::setfill ('0') << std::setw (8) << saved_cloud << ".pcd";
    pcl::io::savePCDFileBinary(name.str(), scene);
}

class CamTracker
{
private:
    typedef pcl::PointXYZRGB PointT;
    boost::shared_ptr<ros::NodeHandle> n_;
    ros::ServiceServer cam_tracker_start_;
    ros::ServiceServer cam_tracker_stop_;
    ros::ServiceServer cam_tracker_vis_compound_;
    ros::ServiceServer cam_tracker_do_ba_;
    ros::ServiceServer cam_tracker_get_tracking_results_;
    ros::ServiceServer cam_tracker_cleanup_;

    ros::Subscriber camera_topic_subscriber_;
    ros::Subscriber camera_info_subscriber_;
    ros::Publisher confidence_publisher_;

    kp::KeypointSlamRGBD2::Parameter param;
    kp::KeypointSlamRGBD2::Ptr camtracker;

    double cos_min_delta_angle_;
    double sqr_min_cam_distance_;
    std::vector<Eigen::Matrix4f> cameras_;
    std::vector<std::pair<int, pcl::PointCloud<pcl::PointXYZRGB>::Ptr> > keyframes_;
    int num_clouds_;
    pcl::PointCloud<PointT>::Ptr scene_;
    int saved_clouds_;
    boost::posix_time::ptime last_cloud_;
    ros::Time last_cloud_ros_time_;
    std::string camera_topic_;
    bool debug_mode_;
    sensor_msgs::CameraInfo camera_info_;
    bool got_camera_info_;
    tf::TransformBroadcaster cameraTransformBroadcaster;

    kp::ProjBundleAdjuster ba;

    void camera_info_cb(const sensor_msgs::CameraInfoPtr& msg) {
      camera_info_ = *msg;
      got_camera_info_=true;
    }

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

        ros::Time start_time_stamp = msg->header.stamp;

        boost::posix_time::ptime start_time = boost::posix_time::microsec_clock::local_time ();
        //std::cout << (start_time - last_cloud_).total_nanoseconds () * 1.0e-9 << std::endl;

        float time_ms = (start_time_stamp - last_cloud_ros_time_).toSec() * 1e3;

        last_cloud_ = start_time;
        last_cloud_ros_time_ = start_time_stamp;

        pcl::ScopeTime t("trackNewCloud");

        if(num_clouds_ < 60)
        {
            num_clouds_++;
            return;
        }

        scene_.reset(new pcl::PointCloud<PointT>);
        pcl::moveFromROSMsg (*msg, *scene_);

        //save point cloud to file
        /*{
          pcl::ScopeTime t("thread creation");
          std::thread (saveToDisk,*scene_, saved_clouds_++).detach();
      }*/

        kp::DataMatrix2D<Eigen::Vector3f> kp_cloud;
        cv::Mat_<cv::Vec3b> image;

        kp::convertCloud(*scene_, kp_cloud, image);

        double conf=0;
        int cam_idx=-1;
        Eigen::Matrix4f pose;

        bool is_ok = camtracker->track(image, kp_cloud, pose, conf, cam_idx);

        if(debug_mode_)
        {
            drawConfidenceBar(image, conf);
            cv::imshow("image", image);
            cv::waitKey(1);
        }

        std::cout << time_ms << " conf:" << conf << std::endl;

        if(is_ok)
        {
            selectFrames(*scene_, cam_idx, pose);
        }

        if(is_ok)
        {
            tf::Transform transform;
            transform.setOrigin(tf::Vector3(pose(0,3), pose(1,3), pose(2,3)));
            tf::Matrix3x3 R(pose(0,0), pose(0,1), pose(0,2),
                            pose(1,0), pose(1,1), pose(1,2),
                            pose(2,0), pose(2,1), pose(2,2));
            tf::Quaternion q;
            R.getRotation(q);
            transform.setRotation(q);
            cameraTransformBroadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "tracked_camera"));
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
    cleanup (camera_srv_definitions::cleanup::Request & req,
             camera_srv_definitions::cleanup::Response & response)
    {
        cameras_.clear();
        keyframes_.clear();
        num_clouds_ = 0;
        saved_clouds_ = 0;
    }

    bool
    start (camera_srv_definitions::start_tracker::Request & req,
           camera_srv_definitions::start_tracker::Response & response)
    {

        cameras_.clear();
        keyframes_.clear();
        num_clouds_ = 0;
        saved_clouds_ = 0;

        camera_topic_subscriber_ = n_->subscribe(camera_topic_ +"/points", 1, &CamTracker::getCloud, this);
        camera_info_subscriber_ = n_->subscribe(camera_topic_ +"/camera_info", 1, &CamTracker::camera_info_cb, this);

        ROS_INFO_STREAM("Wating for camera info...topic=" << camera_topic_ << "/camera_info...");
	while (!got_camera_info_) {
	  ros::Duration(0.1).sleep();
          ros::spinOnce();
	}
        ROS_INFO("got it.");
	camera_info_subscriber_.shutdown();

        cv::Mat_<double> distCoeffs = cv::Mat(4, 1, CV_64F, camera_info_.D.data());
        cv::Mat_<double> intrinsic = cv::Mat(3, 3, CV_64F, camera_info_.K.data()); 
	
        camtracker.reset( new kp::KeypointSlamRGBD2(param) );
        camtracker->setCameraParameter(intrinsic,distCoeffs);


        confidence_publisher_ = n_->advertise<std_msgs::Float32>("cam_tracker_confidence", 1);
        last_cloud_ = boost::posix_time::microsec_clock::local_time ();
        last_cloud_ros_time_ = ros::Time::now();

        return true;
    }

    bool
    stop (camera_srv_definitions::start_tracker::Request & req,
          camera_srv_definitions::start_tracker::Response & response)
    {
        camera_topic_subscriber_.shutdown();
        camtracker->stopObjectManagement();

        return true;
    }


    void createObjectCloudFiltered(pcl::PointCloud<pcl::PointXYZRGB>::Ptr & octree_cloud)
    {

        double max_angle = 70.f;
        double lateral_sigma = 0.0015f;
        bool depth_edges = true;
        float nm_integration_min_weight_ = 0.75f;

        faat_pcl::utils::noise_models::NguyenNoiseModel<pcl::PointXYZRGB> nm;
        std::vector<std::pair<int, pcl::PointCloud<pcl::PointXYZRGB>::Ptr> > &ref_clouds = keyframes_;
        std::vector< std::vector<float> > weights(ref_clouds.size());
        std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr > ptr_clouds(ref_clouds.size());
        std::vector< pcl::PointCloud<pcl::Normal>::Ptr > normals(ref_clouds.size());

        nm.setLateralSigma(lateral_sigma);
        nm.setMaxAngle(max_angle);
        nm.setUseDepthEdges(depth_edges);

        if (ref_clouds.size()>0)
        {

            for (unsigned i=0; i<ref_clouds.size(); i++)
            {
                pcl::NormalEstimationOMP<pcl::PointXYZRGB, pcl::Normal> ne;
                ne.setRadiusSearch(0.01f);
                ne.setInputCloud (ref_clouds[i].second);
                normals[i].reset(new pcl::PointCloud<pcl::Normal>());
                ne.compute (*normals[i]);
            }

            for (unsigned i=0; i<ref_clouds.size(); i++)
            {
                ptr_clouds[i] = ref_clouds[i].second;

                nm.setInputCloud(ref_clouds[i].second);
                nm.setInputNormals(normals[i]);
                nm.compute();
                nm.getWeights(weights[i]);
            }

            faat_pcl::utils::NMBasedCloudIntegration<pcl::PointXYZRGB> nmIntegration;
            octree_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);

            nmIntegration.setInputClouds(ptr_clouds);
            nmIntegration.setWeights(weights);
            nmIntegration.setTransformations(cameras_);
            nmIntegration.setMinWeight(nm_integration_min_weight_);
            nmIntegration.setInputNormals(normals);
            nmIntegration.setMinPointsPerVoxel(1);
            nmIntegration.setResolution(0.005f);
            nmIntegration.setFinalResolution(0.005f);
            nmIntegration.compute(octree_cloud);
        }
    }

    bool
    doBA (camera_srv_definitions::do_ba::Request & req,
                 camera_srv_definitions::do_ba::Response & response)
    {

        if(cameras_.size() == 0)
        {
            ROS_WARN("Called bundle adjusment but no camera poses available\n");
            return false;
        }

        kp::Object &model = camtracker->getModel();


        std::cout << "creating bundle adj. " << std::endl;
        {
            ba.optimize(model);
        }

        std::cout << "destroying bundle adj. " << std::endl;

        for(size_t i=0; i < cameras_.size(); i++)
        {
            Eigen::Matrix4f inv_pose_after_ba;
            std::cout << "keyframe [" << i << "]: " << keyframes_[i].first << std::endl;
            kp::invPose(model.cameras[keyframes_[i].first], inv_pose_after_ba);
            cameras_[i] = inv_pose_after_ba;
        }

        std::cout << "I am done with bundle adjustment. " << std::endl;
        return true;

    }

    bool
    getTrackingResults (camera_srv_definitions::get_tracking_results::Request & req,
                        camera_srv_definitions::get_tracking_results::Response & response)
    {
        for(size_t i=0; i < cameras_.size(); i++)
        {

            sensor_msgs::PointCloud2 msg;
            pcl::toROSMsg(*(keyframes_[i].second), msg);
            response.keyframes.push_back(msg);

            Eigen::Matrix4f trans = cameras_[i];

            geometry_msgs::Transform tt;
            tt.translation.x = trans(0,3);
            tt.translation.y = trans(1,3);
            tt.translation.z = trans(2,3);

            Eigen::Matrix3f rotation = trans.block<3,3>(0,0);
            Eigen::Quaternionf q(rotation);
            tt.rotation.x = q.x();
            tt.rotation.y = q.y();
            tt.rotation.z = q.z();
            tt.rotation.w = q.w();
            response.transforms.push_back(tt);

        }

        return true;

    }

    bool
    visCompound (camera_srv_definitions::visualize_compound::Request & req,
                 camera_srv_definitions::visualize_compound::Response & response)
    {

        if(cameras_.size() == 0)
            return false;

        bool do_ba_ = req.do_ba.data;
        std::cout << "!Number of keyframes: " << cameras_.size() << " " << do_ba_ << std::endl;
        pcl::visualization::PCLVisualizer vis("compound cloud");
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr compound(new pcl::PointCloud<pcl::PointXYZRGB>);

        //Bundle adjustment
        if(do_ba_)
        {
            kp::Object &model = camtracker->getModel();
            ba.optimize(model);

            std::cout << keyframes_.size() << " / cameras size: " << cameras_.size() << std::endl;
            for(size_t i=0; i < cameras_.size(); i++)
            {
                Eigen::Matrix4f inv_pose_after_ba;
                kp::invPose(model.cameras[keyframes_[i].first], inv_pose_after_ba);
                cameras_[i] = inv_pose_after_ba;
                std::cout << cameras_[i] << std::endl << std::endl;
            }
        }

        for(size_t i=0; i < cameras_.size(); i++)
        {
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr c(new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::transformPointCloud( *(keyframes_[i].second), *c, cameras_[i]);
            *compound += *c;
        }

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered;
        createObjectCloudFiltered(filtered);

        int v1, v2;
        vis.createViewPort(0,0,0.5,1,v1);
        vis.createViewPort(0.5,0,1,1,v2);

        vis.addPointCloud(compound, "unfiltered", v1);
        vis.addPointCloud(filtered, "filtered", v2);

        vis.spin();

        return true;

    }

public:
  CamTracker () : got_camera_info_(false)
    {
        cos_min_delta_angle_ = cos(20*M_PI/180.);
        sqr_min_cam_distance_ = 1.*1.;
        num_clouds_ = 0;

        param.det_param.nfeatures = 150;
        param.kt_param.plk_param.use_ncc = true;
        param.kt_param.plk_param.ncc_residual = .5; //   (default .2)

        param.kd_param.rt_param.inl_dist = 0.01; //e.g. 0.01 .. table top, 0.03 ..rooms
        param.lk_param.rt_param.inl_dist = 0.01; //e.g. 0.01 .. table top, 0.03 ..rooms
        param.kt_param.rt_param.inl_dist = 0.03;  //e.g. 0.04 .. table top, 0.1 ..room
        param.om_param.kd_param.rt_param.inl_dist = 0.01; //e.g. 0.01 .. table top, 0.03 ..rooms
        param.om_param.kt_param.rt_param.inl_dist = 0.03;  //e.g. 0.04 .. table top, 0.1 ..room

        camera_topic_ = "/camera/depth_registered;
        debug_mode_ = false;
  }

    void
    initialize (int argc, char ** argv)
    {

        n_.reset( new ros::NodeHandle ( "~" ) );

        cam_tracker_start_  = n_->advertiseService ("start_recording", &CamTracker::start, this);
        cam_tracker_stop_  = n_->advertiseService ("stop_recording", &CamTracker::stop, this);
        cam_tracker_vis_compound_  = n_->advertiseService ("vis_compound", &CamTracker::visCompound, this);
        cam_tracker_do_ba_  = n_->advertiseService ("do_ba", &CamTracker::doBA, this);
        cam_tracker_get_tracking_results_  = n_->advertiseService ("get_results", &CamTracker::getTrackingResults, this);
        cam_tracker_cleanup_  = n_->advertiseService ("cleanup", &CamTracker::cleanup, this);

        if(!n_->getParam ( "camera_topic", camera_topic_ ))
            camera_topic_ = "/camera/depth_registered";

        if(!n_->getParam ( "debug_mode", debug_mode_ ))
            debug_mode_ = false;

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
