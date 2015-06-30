/*
 * main.cpp
 *
 *  Created on: Feb 20, 2014
 *      Author: Thomas Fäulhammer
 */

#include <pcl/common/common.h>
#include <pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "std_msgs/String.h"
#include "recognition_srv_definitions/recognize.h"
#include <v4r/utils/filesystem_utils.h>

class SingleViewRecognizerDemoFromFiles
{
private:
    typedef pcl::PointXYZRGB PointT;
    ros::NodeHandle *n_;
    ros::ServiceClient sv_rec_client;
    std::string directory_,
                models_dir_,
                recognition_structure_dir_;
    bool visualize_;

public:
    bool callSvRecognizer()
    {
        std::vector<std::string> test_cloud;
        v4r::utils::getFilesInDirectory(directory_, test_cloud, "", ".*.pcd", false);
        for(size_t i=0; i < test_cloud.size(); i++)
        {
            pcl::PointCloud<PointT> cloud;
            pcl::io::loadPCDFile(directory_ + "/" + test_cloud[i], cloud);
            sensor_msgs::PointCloud2 cloud_ros;
            pcl::toROSMsg(cloud, cloud_ros);
            recognition_srv_definitions::recognize srv_rec;
            srv_rec.request.cloud = cloud_ros;

            if (!sv_rec_client.call(srv_rec))
            {
                std::stringstream mm;
                mm << "Error calling recognition service. "<< std::endl;
                ROS_ERROR(mm.str().c_str());
                return false;
            }
        }
    }

public:
    bool initialize(int argc, char ** argv)
    {
        ros::init (argc, argv, "SingleViewRecognizerDemoFromFiles");

        n_ = new ros::NodeHandle ( "~" );

        if(!n_->getParam ( "directory", directory_ ))
        {
            //directory_ = "/media/aitor14/DATA/STRANDS_MODELS/recognition_structure/playstation_turn_table.pcd/";
            ROS_ERROR("Specify a directory using param directory.\n");
            exit(-1);
        }

        std::string service_name_sv_rec = "/recognition_service/sv_recognition";
        sv_rec_client = n_->serviceClient<recognition_srv_definitions::recognize>(service_name_sv_rec);
    }
};

int
main (int argc, char ** argv)
{
    SingleViewRecognizerDemoFromFiles m;
    m.initialize(argc, argv);
    m.callSvRecognizer();
    ros::spin();
    return 0;
}
