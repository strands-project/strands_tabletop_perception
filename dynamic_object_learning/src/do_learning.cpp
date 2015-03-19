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

#include "do_learning_srv_definitions/learn_object.h"

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
#include <pcl/search/octree.h>
#include <pcl/segmentation/supervoxel_clustering.h>

class DOL
{
private:
    typedef pcl::PointXYZRGB PointT;
    boost::shared_ptr<ros::NodeHandle> n_;
    ros::ServiceServer learn_object_;

    std::vector<pcl::PointIndices> object_indices_;
    std::vector<Eigen::Matrix4f> cameras_;
    std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > keyframes_;

    ///radius to select points in other frames to belong to the same object
    /// bootstraps region growing
    double radius_;
    double eps_angle_;
    double voxel_resolution_;
    double seed_resolution_;
    double ratio_;

    Eigen::Matrix4f fromGMTransform(geometry_msgs::Transform & gm_trans)
    {
        Eigen::Matrix4f trans = Eigen::Matrix4f::Identity();

        Eigen::Quaternionf q(gm_trans.rotation.w,
                             gm_trans.rotation.x,
                             gm_trans.rotation.y,
                             gm_trans.rotation.z);

        Eigen::Vector3f translation(gm_trans.translation.x,
                                    gm_trans.translation.y,
                                    gm_trans.translation.z);


        trans.block<3,3>(0,0) = q.toRotationMatrix();
        trans.block<3,1>(0,3) = translation;
        return trans;
    }

    void extractEuclideanClustersSmooth (
            const pcl::PointCloud<PointT> &cloud,
            const pcl::PointCloud<pcl::Normal> &normals,
            const pcl::octree::OctreePointCloudSearch<PointT> &tree,
            std::vector<int> &cluster,
            std::set<int> & initial)
    {
        float tolerance = radius_;

        if (tree.getInputCloud ()->points.size () != cloud.points.size ())
        {
            PCL_ERROR ("[pcl::extractEuclideanClusters] Tree built for a different point cloud dataset (%zu) than the input cloud (%zu)!\n", tree.getInputCloud ()->points.size (), cloud.points.size ());
            return;
        }
        if (cloud.points.size () != normals.points.size ())
        {
            PCL_ERROR ("[pcl::extractEuclideanClusters] Number of points in the input point cloud (%zu) different than normals (%zu)!\n", cloud.points.size (), normals.points.size ());
            return;
        }

        // Create a bool vector of processed point indices, and initialize it to false
        std::vector<bool> to_grow (cloud.points.size (), false);
        std::vector<bool> in_cluster (cloud.points.size (), false);

        std::set<int>::iterator it;
        for(it = initial.begin(); it != initial.end(); it++)
        {
            to_grow[*it] = true;
            in_cluster[*it] = true;
        }

        bool stop = false;

        while(!stop)
        {
            std::vector<int> nn_indices;
            std::vector<float> nn_distances;
            // Process all points in the indices vector
            for (int i = 0; i < static_cast<int> (cloud.points.size ()); ++i)
            {
                if (!to_grow[i])
                    continue;

                to_grow[i] = false;

                if (tree.radiusSearch (cloud.points[i], tolerance, nn_indices, nn_distances))
                {
                    for (size_t j = 1; j < nn_indices.size (); ++j) // nn_indices[0] should be the same point
                    {
                        if(!in_cluster[nn_indices[j]])
                        {
                            //check smoothness constraint
                            Eigen::Vector3f n1 = normals.points[i].getNormalVector3fMap();
                            Eigen::Vector3f n2 = normals.points[nn_indices[j]].getNormalVector3fMap();
                            n1.normalize();
                            n2.normalize();
                            float dot_p = n1.dot(n2);
                            if (dot_p >= eps_angle_)
                            {
                                to_grow[nn_indices[j]] = true;
                                in_cluster[nn_indices[j]] = true;
                            }
                        }
                    }
                }
            }

            int ngrow = 0;
            for (size_t i = 0; i < cloud.points.size (); ++i)
            {
                if(to_grow[i])
                    ngrow++;
            }

            if(ngrow == 0)
                stop = true;
        }

        for (size_t i = 0; i < cloud.points.size (); ++i)
        {
            if(in_cluster[i])
                cluster.push_back(static_cast<int>(i));
        }
    }

    void updatePointNormalsFromSuperVoxels(pcl::PointCloud<PointT>::Ptr & cloud,
                                           pcl::PointCloud<pcl::Normal>::Ptr & normals,
                                           std::set<int> & all_neighbours)
    {
        pcl::SupervoxelClustering<PointT> super (voxel_resolution_, seed_resolution_, false);
        super.setInputCloud (cloud);
        super.setColorImportance (0.f);
        super.setSpatialImportance (0.5f);
        super.setNormalImportance (2.f);
        super.setNormalCloud(normals);
        std::map <uint32_t, pcl::Supervoxel<PointT>::Ptr > supervoxel_clusters;
        super.extract (supervoxel_clusters);
        super.refineSupervoxels(2, supervoxel_clusters);

        pcl::console::print_info ("Found %d supervoxels\n", supervoxel_clusters.size ());

        pcl::PointCloud<pcl::PointXYZL>::Ptr supervoxels_labels_cloud = super.getLabeledCloud();
        uint32_t max_label = super.getMaxLabel();

        /*pcl::PointCloud<pcl::PointXYZRGBA>::Ptr sv_cloud = super.getColoredVoxelCloud();
        pcl::visualization::PCLVisualizer vis("sv segmentation");
        vis.addPointCloud(sv_cloud);
        vis.spin();*/

        pcl::PointCloud<pcl::PointNormal>::Ptr sv_normal_cloud = super.makeSupervoxelNormalCloud (supervoxel_clusters);

        std::vector<int> label_to_idx;
        label_to_idx.resize(max_label + 1, -1);
        typename std::map <uint32_t, typename pcl::Supervoxel<PointT>::Ptr>::iterator sv_itr,sv_itr_end;
        sv_itr = supervoxel_clusters.begin ();
        sv_itr_end = supervoxel_clusters.end ();
        int i=0;
        for ( ; sv_itr != sv_itr_end; ++sv_itr, i++)
        {
          label_to_idx[sv_itr->first] = i;
        }

        int sv_size = supervoxel_clusters.size ();

        //count total number of pixels for each supervoxel
        std::vector<int> label_count;
        label_count.resize(sv_size, 0);

        for(size_t i=0; i < supervoxels_labels_cloud->size(); i++)
        {
            int sv_idx = label_to_idx[supervoxels_labels_cloud->at(i).label];
            if(sv_idx < 0 || sv_idx >= sv_size)
                continue;

            Eigen::Vector3f sv_normal = sv_normal_cloud->points[sv_idx].getNormalVector3fMap();
            normals->points[i].getNormalVector3fMap() = sv_normal;
            label_count[sv_idx]++;
        }

        //count for all labels how many pixels are in the initial indices
        std::vector<int> label_count_nn;
        label_count_nn.resize(sv_size, 0);

        std::set<int>::iterator it;
        for(it = all_neighbours.begin(); it != all_neighbours.end(); it++)
        {
            int sv_idx = label_to_idx[supervoxels_labels_cloud->at(*it).label];
            if(sv_idx < 0 || sv_idx >= sv_size)
                continue;

            label_count_nn[sv_idx]++;
        }

        std::set<int> good_neighbours;
        for(it = all_neighbours.begin(); it != all_neighbours.end(); it++)
        {
            int sv_idx = label_to_idx[supervoxels_labels_cloud->at(*it).label];
            if(sv_idx < 0 || sv_idx >= sv_size)
                continue;

            if( (label_count_nn[sv_idx] / (float)label_count[sv_idx]) > ratio_)
            {
                good_neighbours.insert(*it);
            }
        }

        all_neighbours = good_neighbours;
    }

    void transferIndices(int origin, int dest)
    {
        pcl::PointCloud<PointT>::Ptr segmented(new pcl::PointCloud<PointT>);
        pcl::copyPointCloud(*keyframes_[origin], object_indices_[origin], *segmented);

        //transform segmented to dest RF
        Eigen::Matrix4f combined = cameras_[dest].inverse() * cameras_[origin];

        pcl::PointCloud<PointT>::Ptr segmented_trans(new pcl::PointCloud<PointT>);
        pcl::transformPointCloud(*segmented, *segmented_trans, combined);

        //find neighbours from segmented in keyframes_[dest]
        pcl::octree::OctreePointCloudSearch<PointT> octree (0.005f);
        octree.setInputCloud ( keyframes_[dest] );
        octree.addPointsFromInputCloud ();

        std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;

        std::set<int> all_neighbours;

        for(size_t i=0; i < segmented_trans->points.size(); i++)
        {
            if (octree.nearestKSearch (segmented_trans->points[i], 1, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
            //if (octree.radiusSearch (segmented_trans->points[i], radius_, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
            {
                if(pointRadiusSquaredDistance[0] <= (radius_ * radius_))
                all_neighbours.insert(pointIdxRadiusSearch.begin(), pointIdxRadiusSearch.end());
            }
        }

        //smooth region growing
        pcl::PointCloud<pcl::Normal>::Ptr normals_dest(new pcl::PointCloud<pcl::Normal>);
        pcl::NormalEstimation<PointT, pcl::Normal> n3d;
        n3d.setRadiusSearch (0.01f);
        n3d.setInputCloud (keyframes_[dest]);
        n3d.compute (*normals_dest);

        updatePointNormalsFromSuperVoxels(keyframes_[dest], normals_dest, all_neighbours);

        std::cout << "After updatePointNormalsFromSuperVoxels " << std::endl;

        std::vector<int> all_nn(all_neighbours.begin(), all_neighbours.end());
        pcl::copyPointCloud(*keyframes_[dest], all_nn, *segmented_trans);

        /*pcl::visualization::PCLVisualizer vis("transferIndices");
        int v1, v2;
        vis.createViewPort(0,0,0.5,1,v1);
        vis.createViewPort(0.5,0,1,1,v2);
        vis.addPointCloud(keyframes_[dest]);
        pcl::visualization::PointCloudColorHandlerCustom<PointT> handler(segmented_trans, 255, 0, 0);
        vis.addPointCloud(segmented_trans, handler, "segmented_origin", v1);
        vis.spin();*/

        std::vector<int> cluster;
        extractEuclideanClustersSmooth(*keyframes_[dest], *normals_dest,
                                       octree, cluster, all_neighbours);

        std::cout << "size of cluster" << cluster.size() << std::endl;

        /*{
            pcl::PointCloud<PointT>::Ptr segmented_trans(new pcl::PointCloud<PointT>);
            pcl::copyPointCloud(*keyframes_[dest], cluster, *segmented_trans);
            pcl::visualization::PointCloudColorHandlerCustom<PointT> handler(segmented_trans, 255, 0, 0);
            vis.addPointCloud(segmented_trans, handler, "segmented_dest", v2);
        }

        vis.spin();
        */

        object_indices_[dest].indices = cluster;

    }

    void erodeInitialIndices(pcl::PointCloud<PointT> & cloud,
                             std::vector<int> & initial_indices,
                             pcl::PointIndices & eroded_indices)
    {
        cv::Mat mask = cv::Mat(cloud.height, cloud.width, CV_8UC1);
        mask.setTo(0);

        for(size_t i=0; i < initial_indices.size(); i++)
        {
            int r,c;
            r = initial_indices[i] / mask.cols;
            c = initial_indices[i] % mask.cols;

            mask.at<unsigned char>(r,c) = 255;
        }

        /*cv::imshow("mask", mask);
        cv::waitKey(0);*/

        cv::erode(mask, mask, cv::Mat(), cv::Point(-1,-1), 3);

        /*cv::imshow("mask", mask);
        cv::waitKey(0);*/

        for(int r=0; r < mask.rows; r++)
        {
            for(int c=0; c< mask.cols; c++)
            {
                if(mask.at<unsigned char>(r,c) > 0)
                {
                    eroded_indices.indices.push_back(r * mask.cols + c);
                }
            }
        }

    }

    bool
    learn_object (do_learning_srv_definitions::learn_object::Request & req,
                  do_learning_srv_definitions::learn_object::Response & response)
    {

        object_indices_.clear();
        keyframes_.clear();
        cameras_.clear();

        pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
        pcl::fromROSMsg(req.keyframes[0], *cloud);

        pcl::PointIndices pind;
        for(size_t i=0; i < req.intial_object_indices.size(); i++)
        {
            pind.indices.push_back(req.intial_object_indices[i].data);
        }

        //erode mask
        pcl::PointIndices eroded;
        erodeInitialIndices(*cloud, pind.indices, eroded);

        object_indices_.push_back(eroded);

        for(size_t i=0; i < req.keyframes.size(); i++)
        {
            pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
            pcl::fromROSMsg(req.keyframes[i], *cloud);
            keyframes_.push_back(cloud);

            Eigen::Matrix4f t = fromGMTransform(req.transforms[i]);
            cameras_.push_back(t);

        }

        object_indices_.resize(cameras_.size());



        //transfer object indices from initial cloud to the rest incrementally
        //TODO: If the indices are empty, probably means that the point of view does not contain the object, propagate last non-empy indices
        for(int i=1; i < (int)keyframes_.size(); i++)
        {
            transferIndices(i-1, i);
            pcl::PointIndices eroded;
            erodeInitialIndices(*cloud, object_indices_[i].indices, eroded);
            object_indices_[i] = eroded;
        }

        pcl::PointCloud<PointT>::Ptr big_cloud(new pcl::PointCloud<PointT>);
        pcl::PointCloud<PointT>::Ptr big_cloud_segmented(new pcl::PointCloud<PointT>);

        for(size_t i=0; i < req.keyframes.size(); i++)
        {
            pcl::PointCloud<PointT> cloud_trans;
            pcl::transformPointCloud(*keyframes_[i], cloud_trans, cameras_[i]);
            *big_cloud += cloud_trans;

            pcl::PointCloud<PointT> segmented;
            pcl::PointCloud<PointT> segmented_trans;

            pcl::copyPointCloud(*keyframes_[i], object_indices_[i], segmented);
            pcl::transformPointCloud(segmented, segmented_trans, cameras_[i]);
            *big_cloud_segmented += segmented_trans;

        }

        pcl::visualization::PCLVisualizer vis("test");
        int v1, v2;
        vis.createViewPort(0,0,0.5,1,v1);
        vis.createViewPort(0.5,0,1,1,v2);
        vis.addPointCloud(big_cloud, "big", v1);
        vis.addPointCloud(big_cloud_segmented, "segmented", v2);
        vis.spin();

        return true;
    }

public:
    DOL ()
    {
        radius_ = 0.005f;
        eps_angle_ = 0.99f;
        voxel_resolution_ = 0.005f;
        seed_resolution_ = 0.03f;
        ratio_ = 0.25f;
    }

    void
    initialize (int argc, char ** argv)
    {

        n_.reset( new ros::NodeHandle ( "~" ) );

        n_->getParam ( "radius", radius_);
        n_->getParam ( "dot_product", eps_angle_);
        n_->getParam ( "seed_res", seed_resolution_);
        n_->getParam ( "voxel_res", voxel_resolution_);
        n_->getParam ( "ratio", ratio_);

        learn_object_  = n_->advertiseService ("learn_object", &DOL::learn_object, this);
        ros::spin ();
    }
};

int
main (int argc, char ** argv)
{
    ros::init (argc, argv, "dynamic_object_learning");

    DOL m;
    m.initialize (argc, argv);

    return 0;
}
