/*
 * main.cpp
 *
 *  Created on: Feb 20, 2014
 *      Author: Thomas Fäulhammer
 */

#include <pcl/common/common.h>
#include <pcl_conversions.h>
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "do_learning_srv_definitions/learn_object.h"
#include <opencv2/opencv.hpp>
#include <v4r/utils/filesystem_utils.h>
#include <pcl/io/pcd_io.h>

struct IndexPoint
{
  int idx;
};

POINT_CLOUD_REGISTER_POINT_STRUCT (IndexPoint,
(int, idx, idx)
)

class DOLDemoFromFiles
{
private:
    typedef pcl::PointXYZRGB PointT;
    ros::NodeHandle *n_;
    std::string directory_;

public:
    bool callDOL()
    {

        //read files from directory
        std::string so_far = "";

        std::vector<std::string> keyframes_str;
        std::vector<std::string> object_indices_str;
        std::vector<std::string> poses_str;

        {
            std::string pattern = ".*cloud.*.pcd";
            v4r::utils::getFilesInDirectory(directory_, keyframes_str, so_far, pattern, false);
        }

        {
            std::string pattern = ".*object_indices.*.pcd";
            v4r::utils::getFilesInDirectory(directory_, object_indices_str, so_far, pattern, false);
        }

        {
            std::string pattern = ".*pose.*.txt";
            v4r::utils::getFilesInDirectory(directory_, poses_str, so_far, pattern, false);
        }

        std::sort(keyframes_str.begin(), keyframes_str.end());
        std::sort(poses_str.begin(), poses_str.end());
        std::sort(object_indices_str.begin(), object_indices_str.end());

        std::string service_name = "/dynamic_object_learning/learn_object";
        ros::ServiceClient DOLclient = n_->serviceClient<do_learning_srv_definitions::learn_object>(service_name);
        do_learning_srv_definitions::learn_object srv;


        //create request
        std::stringstream str;
        str << directory_ << "/" << object_indices_str[0];

        pcl::PointCloud<IndexPoint> obj_indices_cloud;
        pcl::io::loadPCDFile (str.str(), obj_indices_cloud);

        for(size_t i=0; i < obj_indices_cloud.points.size(); i++)
        {
	   srv.request.intial_object_indices.push_back(obj_indices_cloud.points[i].idx);
        }

        for(size_t i=0; i < keyframes_str.size(); i++)
        {
            pcl::PointCloud<PointT> cloud;
            std::stringstream str;
            str << directory_ << "/" << keyframes_str[i];
            pcl::io::loadPCDFile(str.str(), cloud);

            sensor_msgs::PointCloud2 msg_cloud;
            pcl::toROSMsg(cloud, msg_cloud);

            srv.request.keyframes.push_back(msg_cloud);


            Eigen::Matrix4f trans;

            {
                std::stringstream str;
                str << directory_ << "/" << poses_str[i];
                v4r::utils::readMatrixFromFile (str.str(), trans);
            }

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

            srv.request.transforms.push_back(tt);
        }

        if (DOLclient.call(srv))
        {

        }
        else
        {
            std::stringstream mm;
            mm << "Error calling service: " << service_name << std::endl;
            ROS_ERROR(mm.str().c_str());
            return false;
        }

        return true;
    }

public:
    DOLDemoFromFiles()
    {

    }

    bool initialize(int argc, char ** argv)
    {
        ros::init (argc, argv, "DOLDemoFromFiles");
        if (sizeof(int) != 4)
        {
            ROS_WARN("PC Architectur does not use 32bit for integer - check conflicts with pcl indices.");
        }
        n_ = new ros::NodeHandle ( "~" );

        if(!n_->getParam ( "directory", directory_ ))
            directory_ = "";

        //directory_ = "/media/aitor14/DATA/STRANDS_MODELS/recognition_structure/playstation_turn_table.pcd/";
        if(directory_.compare("") == 0)
        {
            ROS_ERROR("Specify a directory\n");
            exit(-1);
        }
    }
};

int
main (int argc, char ** argv)
{
    DOLDemoFromFiles m;
    m.initialize(argc, argv);
    m.callDOL();
    ros::spin();
    return 0;
}
