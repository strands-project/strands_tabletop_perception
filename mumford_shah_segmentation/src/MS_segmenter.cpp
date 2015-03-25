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
#include "std_msgs/Float32.h"

#include <pcl/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions.h>
#include <pcl/common/angles.h>
#include <pcl/common/transforms.h>
#include <pcl/common/time.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d_omp.h>

#include "segmentation_srv_definitions/MS_segment.h"
#include <v4r/OCTopDownSegmenter/sv_ms_presegmenter.h>
#include <v4r/OCTopDownSegmenter/mv_MS_presegmenter.h>

class MSSegmenter
{
private:
    typedef pcl::PointXYZRGB PointT;
    boost::shared_ptr<ros::NodeHandle> n_;
    ros::ServiceServer segmenter_;
    float radius_normals_;
    float sv_color_, sv_spatial_, sv_normal_;
    float boundary_radius_;
    int boundary_window_;

    void denoisePoint(Eigen::Vector3f p, Eigen::Vector3f n,
                        float sigma_c, float sigma_s,
                        pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
                        int r, int c,
                        pcl::PointCloud<pcl::PointXYZRGB>::Ptr & copy,
                        int kernel_width)
    {
        if(!pcl_isfinite(p[2]))
            return;

        //use neighborhood to compute weights...
        float sum, normalizer;
        sum = normalizer = 0;
        for(int u=std::max(0, r - kernel_width); u <= std::min((int)(cloud->height - 1), r + kernel_width); u++)
        {
            for(int v=std::max(0, c - kernel_width); v <= std::min((int)(cloud->width - 1), c + kernel_width); v++)
            {
                Eigen::Vector3f p_uv = cloud->at(v,u).getVector3fMap();
                if(!pcl_isfinite(p_uv[2]))
                    return;

                float t = (p_uv - p).norm();
                float h = n.dot(p - p_uv);
                float wc, ws;
                wc = std::exp(-t*t / (2*sigma_c*sigma_c));
                ws = std::exp(-h*h / (2*sigma_s*sigma_s));
                sum += wc * ws * h;
                normalizer += wc * ws;
            }
        }
        copy->at(c,r).z = copy->at(c,r).z + (sum / normalizer);
    }

    void bilateral_filter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
                          pcl::PointCloud<pcl::Normal>::Ptr & normals,
                          float sigma_c, float sigma_s,
                          int kernel_width = 5)
    {

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr copy(new pcl::PointCloud<pcl::PointXYZRGB>(*cloud));

        for(int r=0; r < (int)copy->height; r++)
        {
            for(int c=0; c < (int)copy->width; c++)
            {
                denoisePoint(cloud->at(c,r).getVector3fMap(),
                             normals->at(c,r).getNormalVector3fMap(),
                             sigma_c, sigma_s,
                             copy, r, c, cloud, kernel_width);

            }
        }
    }

    bool
    segment (segmentation_srv_definitions::MS_segment::Request & req,
             segmentation_srv_definitions::MS_segment::Response & response)
    {

        pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
        pcl::fromROSMsg(req.cloud, *cloud);

        pcl::PointCloud<pcl::Normal>::Ptr normal_cloud (new pcl::PointCloud<pcl::Normal>);

        pcl::NormalEstimationOMP<pcl::PointXYZRGB, pcl::Normal> ne;
        ne.setRadiusSearch(radius_normals_);
        ne.setInputCloud (cloud);
        ne.compute (*normal_cloud);

        std::vector<std::vector<int> > segmentation_indices;

        if(cloud->isOrganized())
        {
            /*float sigma_s = 0.005f;
            float sigma_c = 0.005f;
            int kernel_width = 5;

            bilateral_filter(cloud, normal_cloud, sigma_s, sigma_c, kernel_width);*/

            bool use_SLIC = req.use_SLIC.data;
            v4rOCTopDownSegmenter::SVMumfordShahPreSegmenter<PointT> pre_segmenter;
            pre_segmenter.setInputCloud(cloud);
            pre_segmenter.setSurfaceNormals(normal_cloud);
            pre_segmenter.setNyu(req.nyu.data);
            pre_segmenter.setLambda(req.lambda.data);
            pre_segmenter.setSigma(0); //color regularizer
            pre_segmenter.setAlpha(1); //data term regularizer

            pre_segmenter.setVisEachMove(false);
            pre_segmenter.setMaxModelType(req.max_mt.data);
            pre_segmenter.setPixelWiseRefinement(req.refinement.data);
            pre_segmenter.setSVParams(req.sv_seed.data, req.sv_res.data);
            pre_segmenter.setSVImportanceValues(sv_color_, sv_spatial_, sv_normal_);
            pre_segmenter.setBoundaryWindow(boundary_window_);
            pre_segmenter.setUseSLIC(use_SLIC);
            pre_segmenter.process();
            pre_segmenter.getSegmentationIndices(segmentation_indices);
        }
        else
        {
            v4rOCTopDownSegmenter::MVMumfordShahPreSegmenter<PointT> pre_segmenter;
            pre_segmenter.setInputCloud(cloud);
            pre_segmenter.setSurfaceNormals(normal_cloud);
            pre_segmenter.setNyu(req.nyu.data);
            pre_segmenter.setLambda(req.lambda.data);
            pre_segmenter.setSigma(0); //color regularizer
            pre_segmenter.setAlpha(1); //data term regularizer

            pre_segmenter.setVisEachMove(false);
            pre_segmenter.setMaxModelType(req.max_mt.data);
            pre_segmenter.setPixelWiseRefinement(false);
            pre_segmenter.setSVParams(req.sv_seed.data, req.sv_res.data);
            pre_segmenter.setSVImportanceValues(sv_color_, sv_spatial_, sv_normal_);
            pre_segmenter.setBoundaryRadius(boundary_radius_);
            pre_segmenter.process();
            pre_segmenter.getSegmentationIndices(segmentation_indices);
        }

        //transform segmentation_indices to srv.response ...
        for(size_t i=0; i < segmentation_indices.size(); i++)
        {
            std_msgs::Int32MultiArray indx;
            response.clusters_indices.push_back(indx);
            response.clusters_indices.back().data = segmentation_indices[i];
        }

        return true;
    }

public:
    MSSegmenter ()
    {
        radius_normals_ = 0.02f;
        boundary_window_ = 1;
        boundary_radius_ = 0.015f;
        sv_color_ = 0.f;
        sv_spatial_ = 1.f;
        sv_normal_ = 3.f;
    }

    void
    initialize (int argc, char ** argv)
    {

        n_.reset( new ros::NodeHandle ( "~" ) );

        segmenter_  = n_->advertiseService ("segment", &MSSegmenter::segment, this);
        ros::spin ();
    }
};

int
main (int argc, char ** argv)
{
    ros::init (argc, argv, "MSSegmenter");

    MSSegmenter m;
    m.initialize (argc, argv);

    return 0;
}
