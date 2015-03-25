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
#include "segmentation_srv_definitions/MS_segment.h"
#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <v4r/ORUtils/pcl_opencv.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>

class Demo
{
private:
    typedef pcl::PointXYZRGB PointT;
    ros::NodeHandle *n_;
    std::string file_;
    bool use_slic_;

    void visualize_output(pcl::PointCloud<PointT>::Ptr & scene,
                          std::vector<std_msgs::Int32MultiArray> & clusters)
    {

        std::vector<uint32_t> label_colors_;

        int max_label = clusters.size();
        if((int)label_colors_.size() != max_label)
        {
            label_colors_.reserve (max_label + 1);
            srand (static_cast<unsigned int> (time (0)));
            while ((int)label_colors_.size () <= max_label )
            {
                uint8_t r = static_cast<uint8_t>( (rand () % 256));
                uint8_t g = static_cast<uint8_t>( (rand () % 256));
                uint8_t b = static_cast<uint8_t>( (rand () % 256));
                label_colors_.push_back (static_cast<uint32_t>(r) << 16 | static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
            }
        }

        if(scene->isOrganized())
        {
            cv::Mat_<cv::Vec3b> image;
            PCLOpenCV::ConvertPCLCloud2Image<pcl::PointXYZRGB>(scene, image);

            cv::Mat image_clone = image.clone();

            float factor = 0.05f;

            for(size_t i=0; i < clusters.size(); i++)
            {
                for(size_t j=0; j < clusters[i].data.size(); j++)
                {
                    int r, c;
                    int idx = clusters[i].data[j];
                    r = idx / scene->width;
                    c = idx % scene->width;

                    uint32_t rgb = label_colors_[i];
                    unsigned char rs = (rgb >> 16) & 0x0000ff;
                    unsigned char gs = (rgb >> 8) & 0x0000ff;
                    unsigned char bs = (rgb) & 0x0000ff;

                    cv::Vec3b im = image.at<cv::Vec3b>(r,c);
                    image.at<cv::Vec3b>(r,c) = cv::Vec3b((unsigned char)(im[0] * factor + bs * (1 - factor)),
                            (unsigned char)(im[1] * factor + gs * (1 - factor)),
                            (unsigned char)(im[2] * factor + rs * (1 - factor)));
                }
            }

            cv::Mat collage = cv::Mat(image.rows, image.cols * 2, CV_8UC3);
            collage.setTo(cv::Vec3b(0,0,0));

            for(unsigned int r=0; r < scene->height; r++)
            {
                for(unsigned int c=0; c < scene->width; c++)
                {
                    collage.at<cv::Vec3b>(r,c) = image_clone.at<cv::Vec3b>(r,c);
                }
            }

            collage(cv::Range(0, collage.rows), cv::Range(collage.cols/2, collage.cols)) = image + cv::Scalar(cv::Vec3b(0, 0, 0));

            cv::imshow("regions", collage);
            cv::waitKey(0);
        }
        else
        {

            //same thing with point cloud
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cc(new pcl::PointCloud<pcl::PointXYZRGB>(*scene));
            for(size_t i=0; i < clusters.size(); i++)
            {
                for(size_t j=0; j < clusters[i].data.size(); j++)
                {
                    cloud_cc->at(clusters[i].data[j]).rgb = label_colors_[i];
                }
            }

            pcl::visualization::PCLVisualizer vis("regions");
            vis.addPointCloud(cloud_cc);
            vis.spin();

        }
    }

public:
    bool call()
    {

        pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
        pcl::io::loadPCDFile(file_, *cloud);

        if(!cloud->isOrganized())
        {
            pcl::VoxelGrid<pcl::PointXYZRGB> filter;
            filter.setInputCloud(cloud);
            filter.setDownsampleAllData(true);
            filter.setLeafSize(0.01f,0.01f,0.01f);

            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_voxel(new pcl::PointCloud<pcl::PointXYZRGB>);
            filter.filter(*cloud_voxel);
            cloud = cloud_voxel;

        }

        sensor_msgs::PointCloud2 msg_cloud;
        pcl::toROSMsg(*cloud, msg_cloud);

        std::string service_name = "/MSSegmenter/segment";
        ros::ServiceClient DOLclient = n_->serviceClient<segmentation_srv_definitions::MS_segment>(service_name);
        segmentation_srv_definitions::MS_segment srv;
        srv.request.cloud = msg_cloud;
        srv.request.max_mt.data = 1;
        srv.request.refinement.data = true;

        if(cloud->isOrganized())
        {
            srv.request.use_SLIC.data = use_slic_;
            srv.request.nyu.data = 0.015;
            srv.request.lambda.data = 0;
            srv.request.sv_res.data = 0.004f;
            srv.request.sv_seed.data = 0.03;
        }
        else
        {
            srv.request.nyu.data = 0.25;
            srv.request.lambda.data = 0.00001;
            srv.request.sv_res.data = 0.025f;
            srv.request.sv_seed.data = 0.15;
        }

        if (DOLclient.call(srv))
        {
            //visualize stuff from srv.response
            std::cout << "Number of clusters:" << srv.response.clusters_indices.size() << std::endl;
            visualize_output(cloud, srv.response.clusters_indices);
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
    Demo()
    {
        use_slic_ = false;
    }

    bool initialize(int argc, char ** argv)
    {
        ros::init (argc, argv, "MSDemo");
        n_ = new ros::NodeHandle ( "~" );

        if(!n_->getParam ( "file", file_ ))
            file_ = "";

        if(file_.compare("") == 0)
        {
            ROS_ERROR("Specify a file");
            exit(-1);
        }

        if(!n_->getParam ( "slic", use_slic_ ))
            use_slic_ = false;
    }
};

int
main (int argc, char ** argv)
{
    Demo m;
    m.initialize(argc, argv);
    m.call();
    ros::spin();
    return 0;
}
