/*
 * main.cpp
 *
 *  Created on: Feb 20, 2014
 *      Author: Thomas Fäulhammer
 */

#include <pcl/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions.h>
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "recognition_srv_definitions/recognize.h"
#include "recognition_srv_definitions/retrain_recognizer.h"
#include "v4r/ORFramework/model_only_source.h"

class SOCDemo
{
private:
    typedef pcl::PointXYZ PointT;
    int kinect_trials_;
    int service_calls_;
    std::string topic_;
    bool KINECT_OK_;
    bool all_required_services_okay_;
    ros::NodeHandle *n_;
    bool visualize_output_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene_;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> vis_;
    std::string models_dir_;
    boost::shared_ptr < faat_pcl::rec_3d_framework::ModelOnlySource<pcl::PointXYZRGBNormal, pcl::PointXYZRGB>
            > models_source_;

    std::vector<std::string> model_ids_;
    std::vector<Eigen::Matrix4f> transforms_;

    bool new_models_added_;

    void
    checkCloudArrive (const sensor_msgs::PointCloud2::ConstPtr& msg)
    {
        KINECT_OK_ = true;
    }

    void
    checkKinect ()
    {
        ros::Subscriber sub_pc = n_->subscribe (topic_, 1, &SOCDemo::checkCloudArrive, this);
        ros::Rate loop_rate (1);
        kinect_trials_ = 0;
        while (!KINECT_OK_ && ros::ok ())
        {
            std::cout << "Checking kinect status..." << std::endl;
            ros::spinOnce ();
            loop_rate.sleep ();
            kinect_trials_++;
            if(kinect_trials_ >= 30)
            {
                std::cout << "Kinect is not working..." << std::endl;
                return;
            }
        }

        KINECT_OK_ = true;
        std::cout << "Kinect is up and running, new_models:" << new_models_added_ << std::endl;

        if(new_models_added_)
        {
            std::cout << "NEW MODELS HAVE BEEN ADDED... call retrain..." << std::endl;

            ros::ServiceClient retrainClient = n_->serviceClient<recognition_srv_definitions::retrain_recognizer>("/recognition_service/mp_recognition_retrain");
            recognition_srv_definitions::retrain_recognizer srv;
            if(retrainClient.call(srv))
            {
                std::cout << "called retrain succesfull" << std::endl;
            }
            else
            {
                ROS_ERROR("Failed to call /recognition_service/mp_recognition_retrain");
                exit(-1);
            }
        }
    }

    bool callSegAndClassifierService(const sensor_msgs::PointCloud2::ConstPtr& msg)
    {
        ros::ServiceClient segAndClassifierClient = n_->serviceClient<recognition_srv_definitions::recognize>("/recognition_service/mp_recognition");
        recognition_srv_definitions::recognize srv;
        srv.request.cloud = *msg;
        if (segAndClassifierClient.call(srv))
        {
            model_ids_.clear();
            transforms_.clear();

            for(size_t i=0; i < srv.response.ids.size(); i++)
            {
                model_ids_.push_back(srv.response.ids[i].data);

                Eigen::Quaternionf q(srv.response.transforms[i].rotation.w,
                                     srv.response.transforms[i].rotation.x,
                                     srv.response.transforms[i].rotation.y,
                                     srv.response.transforms[i].rotation.z);

                Eigen::Vector3f translation(srv.response.transforms[i].translation.x,
                                            srv.response.transforms[i].translation.y,
                                            srv.response.transforms[i].translation.z);


                Eigen::Matrix4f trans;
                trans.block<3,3>(0,0) = q.toRotationMatrix();
                trans.block<3,1>(0,3) = translation;
                transforms_.push_back(trans);
            }

            std::cout << "Called done..." << std::endl;
        }
        else
        {
            ROS_ERROR("Failed to call /recognition_service/mp_recognition");
            return false;
        }
        return true;
    }

    void
    callService (const sensor_msgs::PointCloud2::ConstPtr& msg)
    {
      std::cout << "Received point cloud.\n" << std::endl;
        // if any service is not available, wait for 5 sec and check again
        if( all_required_services_okay_ || ( !all_required_services_okay_ && (service_calls_ % (1 * 5)) == 0))
        {
            std::cout << "going to call service..." << std::endl;

            all_required_services_okay_ = callSegAndClassifierService(msg);

            if (visualize_output_ && all_required_services_okay_)
            {
                pcl::fromROSMsg(*msg, *scene_);
                visualize_output();
            }
        }
        service_calls_++;

    }
public:
    SOCDemo()
    {
        scene_.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
        KINECT_OK_ = false;
        topic_ = "/camera/depth_registered/points";
        kinect_trials_ = 5;
        all_required_services_okay_ = false;
        new_models_added_ = false;
        visualize_output_ = false;
        models_dir_ = "";
    }

    bool initialize(int argc, char ** argv)
    {
        ros::init (argc, argv, "classifier_demo");
        if (sizeof(int) != 4)
        {
            ROS_WARN("PC Architectur does not use 32bit for integer - check conflicts with pcl indices.");
        }
        n_ = new ros::NodeHandle ( "~" );

        if(!n_->getParam ( "topic", topic_ ))
            topic_ = "/camera/depth_registered/points";

        if(!n_->getParam ( "visualize_output", visualize_output_ ))
            visualize_output_ = false;

        if(!n_->getParam ( "models_dir", models_dir_ ))
            models_dir_ = "";

        if(!n_->getParam ( "new_models", new_models_added_ ))
            new_models_added_ = false;

        checkKinect();

        if(models_dir_.compare("") != 0)
        {
            models_source_.reset (new faat_pcl::rec_3d_framework::ModelOnlySource<pcl::PointXYZRGBNormal, pcl::PointXYZRGB>);
            models_source_->setPath (models_dir_);
            models_source_->setLoadViews (false);
            models_source_->setModelScale(1);
            models_source_->setLoadIntoMemory(false);

            std::string test = "irrelevant";
            models_source_->setExtension("pcd");
            models_source_->generate (test);
        }

        return KINECT_OK_;
    }

    void run()
    {
        ros::Subscriber sub_pc = n_->subscribe (topic_, 1, &SOCDemo::callService, this);
        ros::spin();
    }

    void visualize_output()
    {
        if(!vis_)
        {
            vis_.reset(new pcl::visualization::PCLVisualizer("classifier visualization"));
        }

        int v1, v2;
        vis_->createViewPort(0,0,0.5,1,v1);
        vis_->createViewPort(0.5,0,1,1,v2);

        vis_->addCoordinateSystem(0.2f);

        std::cout << scene_->points.size() << std::endl;
        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> handler(scene_);
        vis_->addPointCloud(scene_, handler, "scene", v1);

        std::cout << models_dir_ << " " << transforms_.size() << std::endl;

        if(models_dir_.compare("") != 0 && transforms_.size() > 0)
        {
            //show models

            for(size_t kk=0; kk < transforms_.size(); kk++)
            {
                boost::shared_ptr<faat_pcl::rec_3d_framework::Model<pcl::PointXYZRGB> > model;

                models_source_->getModelById(model_ids_[kk], model);
                pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr model_cloud = model->getAssembled (0.003f);
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr model_aligned (new pcl::PointCloud<pcl::PointXYZRGB>);
                pcl::transformPointCloud (*model_cloud, *model_aligned, transforms_[kk]);

                std::stringstream name;
                name << "hypotheses_" << kk;

                pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> handler(model_aligned);
                vis_->addPointCloud<pcl::PointXYZRGB> (model_aligned, handler, name.str (), v2);

                std::cout << "adding " << name.str() << std::endl;
            }
        }
        vis_->spin();

        vis_->removeAllPointClouds();
    }
};

int
main (int argc, char ** argv)
{
    SOCDemo m;
    m.initialize (argc, argv);
    m.run();
    return 0;
}
