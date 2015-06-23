/*
 * shape_simple_classifier_node.cpp
 *
 *  Created on: Sep 7, 2013
 *      Author: aitor
 */

#define EIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET

#include <thread>
#include "sensor_msgs/PointCloud2.h"
#include "std_msgs/Float32.h"
#include <stdlib.h>

#include <pcl_conversions.h>
#include <pcl/common/angles.h>
#include <pcl/common/transforms.h>
#include <pcl/common/time.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d_omp.h>

#include "do_learning.h"

#include "v4r/KeypointTools/invPose.hpp"
#include "v4r/KeypointTools/toString.hpp"
#include "v4r/KeypointTools/PoseIO.hpp"
#include "v4r/KeypointSlam/KeypointSlamRGBD2.hh"
#include "v4r/KeypointSlam/ProjBundleAdjuster.hh"
#include "v4r/KeypointTools/PointTypes.hpp"
#include "v4r/KeypointTools/ScopeTime.hpp"
#include <v4r/ORUtils/noise_model_based_cloud_integration.h>
#include <v4r/ORUtils/noise_models.h>
#include <pcl/segmentation/supervoxel_clustering.h>
#include <v4r/utils/filesystem_utils.h>
#include <v4r/ORUtils/pcl_visualization_utils.h>
#include <pcl/filters/statistical_outlier_removal.h>

#define NUM_SUBWINDOWS 6

void DOL::extractEuclideanClustersSmooth (
        const pcl::PointCloud<PointT> &cloud,
        const pcl::PointCloud<pcl::Normal> &normals,
        const pcl::octree::OctreePointCloudSearch<PointT> &tree,
        const std::vector<int> &initial,
        std::vector<int> &cluster)
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

    std::vector<int>::const_iterator it;
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
        for (size_t i = 0; i < cloud.points.size (); i++)
        {
            if (!to_grow[i])
                continue;

            to_grow[i] = false;

            if (tree.radiusSearch (cloud.points[i], tolerance, nn_indices, nn_distances))
            {
                for (size_t j = 0; j < nn_indices.size (); ++j) // nn_indices[0] should be the same point
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

        size_t ngrow = 0;
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

void DOL::updatePointNormalsFromSuperVoxels(pcl::PointCloud<PointT>::Ptr & cloud,
                                            pcl::PointCloud<pcl::Normal>::Ptr & normals,
                                            const std::vector<int> & all_neighbours,
                                            std::vector<int> & good_neighbours,
                                            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &supervoxel_cloud)
{
    good_neighbours.clear();

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

    const pcl::PointCloud<pcl::PointXYZL>::Ptr supervoxels_labels_cloud = super.getLabeledCloud();
    uint32_t max_label = super.getMaxLabel();

    supervoxel_cloud = super.getColoredVoxelCloud();
    //        pcl::visualization::PCLVisualizer vis("sv segmentation");
    //        vis.addPointCloud(sv_cloud);
    //        vis.spinOnce();

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

    for(size_t id = 0; id < all_neighbours.size(); id++)
    {
        int sv_idx = label_to_idx[ supervoxels_labels_cloud->at( all_neighbours[ id ] ).label];
        if(sv_idx >= 0 && sv_idx < sv_size)
        {
            label_count_nn[sv_idx]++;
        }
    }

    for(size_t id = 0; id < all_neighbours.size(); id++)
    {
        int sv_idx = label_to_idx[ supervoxels_labels_cloud->at( all_neighbours[ id ] ).label];
        if(sv_idx < 0 || sv_idx >= sv_size)
            continue;

        if( (label_count_nn[sv_idx] / (float)label_count[sv_idx]) > ratio_)
        {
            good_neighbours.push_back( all_neighbours[ id ] );
        }
    }
}

void DOL::transferIndicesAndNNSearch(size_t origin, size_t dest, std::vector<int> &nn)
{
    pcl::PointCloud<PointT>::Ptr segmented(new pcl::PointCloud<PointT>);
    if (do_erosion_)
    {
        pcl::copyPointCloud(*keyframes_[origin], object_indices_eroded_[origin], *segmented);
    }
    else
    {
        pcl::copyPointCloud(*keyframes_[origin], object_indices_[origin], *segmented);
    }

    //transform segmented to dest RF
    Eigen::Matrix4f combined = cameras_[dest].inverse() * cameras_[origin];
    //Eigen::Matrix4f combined = cameras_[dest] * cameras_[origin].inverse();

    pcl::PointCloud<PointT>::Ptr segmented_trans(new pcl::PointCloud<PointT>);
    pcl::transformPointCloud(*segmented, *segmented_trans, combined);

    transferred_cluster_[dest] = segmented_trans;

    //find neighbours from segmented in keyframes_[dest]
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;

    std::set<int> all_neighbours;

    for(size_t i=0; i < segmented_trans->points.size(); i++)
    {
//        if (octree.nearestKSearch (segmented_trans->points[i], 1, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
//            //if (octree.radiusSearch (segmented_trans->points[i], radius_, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
//        {
//            if(pointRadiusSquaredDistance[0] <= (radius_ * radius_))
//                all_neighbours.insert(pointIdxRadiusSearch.begin(), pointIdxRadiusSearch.end());
//        }
        if (octree.radiusSearch (segmented_trans->points[i], radius_, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
        {
             all_neighbours.insert(pointIdxRadiusSearch.begin(), pointIdxRadiusSearch.end());
        }
    }

    std::copy(all_neighbours.begin(), all_neighbours.end(), std::back_inserter(nn));

    std::cout << "Found nearest neighbor: " << nn.size() << std::endl;

//    boost::shared_ptr<pcl::visualization::PCLVisualizer> vis_nn;
//    vis_nn.reset(new pcl::visualization::PCLVisualizer());
//    std::vector<std::string> subwindow_title;
//    subwindow_title.push_back("original scene");
//    subwindow_title.push_back("search points");
//    std::vector<int> vp_nn;
//    vp_nn = faat_pcl::utils::visualization_framework (vis_nn, 1, 2, subwindow_title);
//    pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb_handler(octree.getInputCloud());
//    vis_nn->addPointCloud(octree.getInputCloud(), rgb_handler, "input_cloud", vp_nn[0]);
//    pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb_handler2(segmented_trans);
//    vis_nn->addPointCloud(segmented_trans, rgb_handler2, "segmented_trans", vp_nn[1]);
//    vis_nn->spinOnce();
}

void DOL::erodeInitialIndices(const pcl::PointCloud<PointT> & cloud,
                              const pcl::PointIndices & initial_indices,
                              pcl::PointIndices & eroded_indices)
{
    cv::Mat mask = cv::Mat(cloud.height, cloud.width, CV_8UC1);
    cv::Mat mask_dst;
    mask.setTo(0);

    for(size_t i=0; i < initial_indices.indices.size(); i++)
    {
        int r,c;
        r = initial_indices.indices[i] / mask.cols;
        c = initial_indices.indices[i] % mask.cols;

        mask.at<unsigned char>(r,c) = 255;
    }

    cv::Mat const structure_elem = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
    cv::Mat close_result;
    cv::morphologyEx(mask, close_result, cv::MORPH_CLOSE, structure_elem);

    cv::erode(close_result, mask_dst, cv::Mat(), cv::Point(-1,-1), 3);

//    cv::imshow("mask", mask);
//    cv::imshow("close_result", close_result);
//    cv::imshow("mask_dst", mask_dst);
//    cv::waitKey(0);

    eroded_indices.indices.clear();
    for(int r=0; r < mask_dst.rows; r++)
    {
        for(int c=0; c< mask_dst.cols; c++)
        {
            if(mask_dst.at<unsigned char>(r,c) > 0)
            {
                eroded_indices.indices.push_back(r * mask_dst.cols + c);
            }
        }
    }

}


bool
DOL::save_model (do_learning_srv_definitions::save_model::Request & req,
                 do_learning_srv_definitions::save_model::Response & response)
{
    std::string models_dir = req.models_folder.data;
    std::string recognition_structure_dir = req.recognition_structure_folder.data;
    std::string model_name = req.object_name.data;

    std::vector<pcl::PointCloud<IndexPoint> > object_indices_clouds;

    std::vector<std::vector<float> > weights;
    std::vector<pcl::PointCloud<pcl::Normal>::Ptr > normals;
    std::vector<std::vector<int> > indices;

    weights.resize(keyframes_.size());
    normals.resize(keyframes_.size());
    indices.resize(keyframes_.size());
    object_indices_clouds.resize(keyframes_.size());

    //compute normals for all clouds
    for(size_t i=0; i < keyframes_.size(); i++)
    {
        normals[i].reset(new pcl::PointCloud<pcl::Normal>);
        pcl::NormalEstimation<PointT, pcl::Normal> n3d;
        n3d.setRadiusSearch (0.015f);
        n3d.setInputCloud (keyframes_[i]);
        n3d.compute (*normals[i]);

        indices[i] = object_indices_eroded_[i].indices;

        pcl::PointCloud<IndexPoint> obj_indices_cloud;
        object_indices_clouds[i].points.resize(indices[i].size());

        for(size_t k=0; k < indices[i].size(); k++)
        {
            object_indices_clouds[i].points[k].idx = indices[i][k];
        }
    }

    //compute noise weights
    for(size_t i=0; i < keyframes_.size(); i++)
    {
        faat_pcl::utils::noise_models::NguyenNoiseModel<pcl::PointXYZRGB> nm;
        nm.setInputCloud(keyframes_[i]);
        nm.setInputNormals(normals[i]);
        nm.setLateralSigma(0.001);
        nm.setMaxAngle(60.f);
        nm.setUseDepthEdges(true);
        nm.compute();
        nm.getWeights(weights[i]);
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr octree_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    faat_pcl::utils::NMBasedCloudIntegration<pcl::PointXYZRGB> nmIntegration;
    nmIntegration.setInputClouds(keyframes_);
    nmIntegration.setResolution(0.002f);
    nmIntegration.setWeights(weights);
    nmIntegration.setTransformations(cameras_);
    nmIntegration.setMinWeight(0.5f);
    nmIntegration.setInputNormals(normals);
    nmIntegration.setMinPointsPerVoxel(1);
    nmIntegration.setFinalResolution(0.002f);
    nmIntegration.setIndices(indices);
    nmIntegration.setThresholdSameSurface(0.01f);
    nmIntegration.compute(octree_cloud);

    pcl::PointCloud<pcl::Normal>::Ptr octree_normals;
    nmIntegration.getOutputNormals(octree_normals);

    createDirIfNotExist(recognition_structure_dir);
    createDirIfNotExist(models_dir);

    std::stringstream export_to_rs;
    export_to_rs << recognition_structure_dir << "/" << model_name << "/";
    std::string export_to = export_to_rs.str();

    createDirIfNotExist(export_to);

    //save the data with new poses
    for(size_t i=0; i < cameras_.size(); i++)
    {
        std::stringstream view_file;
        view_file << export_to << "/cloud_" << setfill('0') << setw(8) << i << ".pcd";

        pcl::io::savePCDFileBinary (view_file.str (), *(keyframes_[i]));
        std::cout << view_file.str() << std::endl;

        std::string file_replaced1 (view_file.str());
        boost::replace_last (file_replaced1, "cloud", "pose");
        boost::replace_last (file_replaced1, ".pcd", ".txt");

        std::cout << file_replaced1 << std::endl;

        //read pose as well
        v4r::utils::writeMatrixToFile(file_replaced1, cameras_[i]);

        std::string file_replaced2 (view_file.str());
        boost::replace_last (file_replaced2, "cloud", "object_indices");

        std::cout << file_replaced2 << std::endl;

        pcl::io::savePCDFileBinary (file_replaced2, object_indices_clouds[i]);
    }

    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr filtered_with_normals_oriented (new pcl::PointCloud<pcl::PointXYZRGBNormal>());

    std::stringstream model_output;
    model_output << models_dir << "/" << model_name;
    pcl::concatenateFields(*octree_normals, *octree_cloud, *filtered_with_normals_oriented);

    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_normals_oriented (new pcl::PointCloud<pcl::PointXYZRGBNormal>());

    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGBNormal> sor;
    sor.setInputCloud (filtered_with_normals_oriented);
    sor.setMeanK (50);
    sor.setStddevMulThresh (3.0);
    sor.filter (*cloud_normals_oriented);

    pcl::io::savePCDFileBinary(model_output.str(), *cloud_normals_oriented);
    return true;
}

bool
DOL::learn_object (do_learning_srv_definitions::learn_object::Request & req,
                   do_learning_srv_definitions::learn_object::Response & response)
{
    assert(req.transforms.size() == req.keyframes.size());
    clearMem();
    reserveMem(req.keyframes.size());

    if (visualize_ && !vis_) {
        vis_.reset(new pcl::visualization::PCLVisualizer());
        std::vector<std::string> subwindow_title;
        subwindow_title.push_back("original scene");
        subwindow_title.push_back("supervoxelled scene");
        subwindow_title.push_back("after nearest neighbor search");
        subwindow_title.push_back("good points");
        subwindow_title.push_back("before 2D erosion");
        subwindow_title.push_back("after 2D erosion");
        vis_viewpoint_ = faat_pcl::utils::visualization_framework (vis_, req.keyframes.size(), NUM_SUBWINDOWS, subwindow_title);
    }

    for(size_t i=0; i < req.intial_object_indices.size(); i++)
    {
        transferred_object_indices_[0].indices.push_back(req.intial_object_indices[i].data);
    }

    for(size_t i=0; i < req.keyframes.size(); i++)
    {
        pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
        pcl::fromROSMsg(req.keyframes[i], *cloud);
        keyframes_[i] = cloud;
        cameras_[i] = fromGMTransform(req.transforms[i]);
    }

    //erode mask
    erodeInitialIndices(*keyframes_[0], transferred_object_indices_[0], object_indices_eroded_[0]);
    object_indices_[0] = transferred_object_indices_[0];

    pcl::PointCloud<PointT>::Ptr big_cloud(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr big_cloud_segmented(new pcl::PointCloud<PointT>);

    //transfer object indices from initial cloud to the rest incrementally
    //TODO: If the indices are empty, probably means that the point of view does not contain the object, propagate last non-empy indices
    for(size_t i=0; i < keyframes_.size(); i++)
    {
        std::cout << "Computing indices for cloud " << i << std::endl
                  << "===================================" << std::endl;

        octree.setInputCloud ( keyframes_[i] );
        octree.addPointsFromInputCloud ();

        if (i>=1)
        {
            transferIndicesAndNNSearch(i-1, i, transferred_object_indices_[i].indices);
        }

        //smooth region growing
        pcl::PointCloud<pcl::Normal>::Ptr normals_dest(new pcl::PointCloud<pcl::Normal>);
        pcl::NormalEstimation<PointT, pcl::Normal> n3d;
        n3d.setRadiusSearch (0.01f);
        n3d.setInputCloud (keyframes_[i]);
        n3d.compute (*normals_dest);

        updatePointNormalsFromSuperVoxels(keyframes_[i],
                                          normals_dest,
                                          transferred_object_indices_[i].indices,
                                          transferred_object_indices_good_[i].indices,
                                          supervoxeled_clouds_[i]);

        std::vector<int> cluster;
        extractEuclideanClustersSmooth(*keyframes_[i], *normals_dest, octree, transferred_object_indices_good_[i].indices, cluster);

        object_indices_[i].indices = cluster;

        erodeInitialIndices(*keyframes_[i], object_indices_[i], object_indices_eroded_[i]);

        std::cout << "Found " << transferred_object_indices_[i].indices.size() << " nearest neighbors before growing." << std::endl
                  << "After updatePointNormalsFromSuperVoxels size: " << transferred_object_indices_good_[i].indices.size() << std::endl
                  << "size of final cluster: " << cluster.size() << std::endl
                  << "size of final cluster after erosion: " << object_indices_eroded_[i].indices.size() << std::endl << std::endl;

        pcl::PointCloud<PointT>::Ptr cloud_trans (new pcl::PointCloud<PointT>());
        pcl::transformPointCloud(*keyframes_[i], *cloud_trans, cameras_[i]);
        *big_cloud += *cloud_trans;

        pcl::PointCloud<PointT>::Ptr segmented_eroded (new pcl::PointCloud<PointT>());
        pcl::PointCloud<PointT>::Ptr segmented_eroded_trans (new pcl::PointCloud<PointT>());
        pcl::copyPointCloud(*keyframes_[i], object_indices_eroded_[i], *segmented_eroded);
        pcl::transformPointCloud(*segmented_eroded, *segmented_eroded_trans, cameras_[i]);
        *big_cloud_segmented += *segmented_eroded_trans;
    }

    if(visualize_)
    {
        pcl::visualization::PCLVisualizer vis("segmented cloud");
        int v1, v2;
        vis.createViewPort(0,0,0.5,1,v1);
        vis.createViewPort(0.5,0,1,1,v2);
        vis.addPointCloud(big_cloud, "big", v1);
        vis.addPointCloud(big_cloud_segmented, "segmented", v2);
        vis.spinOnce();

        visualize();        
    }

    return true;
}

void DOL::initialize (int argc, char ** argv)
{
    n_.reset( new ros::NodeHandle ( "~" ) );
    n_->getParam ( "radius", radius_);
    n_->getParam ( "dot_product", eps_angle_);
    n_->getParam ( "seed_res", seed_resolution_);
    n_->getParam ( "voxel_res", voxel_resolution_);
    n_->getParam ( "ratio", ratio_);
    n_->getParam ( "visualize", visualize_);
    n_->getParam ( "do_erosion", do_erosion_);

    learn_object_  = n_->advertiseService ("learn_object", &DOL::learn_object, this);
    save_model_  = n_->advertiseService ("save_model", &DOL::save_model, this);

    std::cout << "Started dynamic object learning with parameters: " << std::endl
              << "===================================================" << std::endl
              << "radius: " << radius_ << std::endl
              << "eps_angle: " << eps_angle_ << std::endl
              << "voxel_resolution: " << seed_resolution_ << std::endl
              << "ratio: " << ratio_ << std::endl
              << "===================================================" << std::endl << std::endl;

    ros::spin ();
}

void DOL::visualize()
{
    for(size_t i=0; i < keyframes_.size(); i++)
    {
        pcl::PointCloud<PointT>::Ptr cloud_trans (new pcl::PointCloud<PointT>());
        pcl::transformPointCloud(*keyframes_[i], *cloud_trans, cameras_[i]);

        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr sv_trans (new pcl::PointCloud<pcl::PointXYZRGBA>());
        pcl::transformPointCloud(*supervoxeled_clouds_[i], *sv_trans, cameras_[i]);

        pcl::PointCloud<PointT>::Ptr segmented (new pcl::PointCloud<PointT>());
        pcl::PointCloud<PointT>::Ptr segmented_trans (new pcl::PointCloud<PointT>());
        pcl::copyPointCloud(*keyframes_[i], transferred_object_indices_[i], *segmented);
        pcl::transformPointCloud(*segmented, *segmented_trans, cameras_[i]);

        pcl::PointCloud<PointT>::Ptr segmented2 (new pcl::PointCloud<PointT>());
        pcl::PointCloud<PointT>::Ptr segmented2_trans (new pcl::PointCloud<PointT>());
        pcl::copyPointCloud(*keyframes_[i], transferred_object_indices_good_[i], *segmented2);
        pcl::transformPointCloud(*segmented2, *segmented2_trans, cameras_[i]);

        pcl::PointCloud<PointT>::Ptr segmented3 (new pcl::PointCloud<PointT>());
        pcl::PointCloud<PointT>::Ptr segmented3_trans (new pcl::PointCloud<PointT>());
        pcl::copyPointCloud(*keyframes_[i], object_indices_[i], *segmented3);
        pcl::transformPointCloud(*segmented3, *segmented3_trans, cameras_[i]);

        pcl::PointCloud<PointT>::Ptr segmented_eroded (new pcl::PointCloud<PointT>());
        pcl::PointCloud<PointT>::Ptr segmented_eroded_trans (new pcl::PointCloud<PointT>());
        pcl::copyPointCloud(*keyframes_[i], object_indices_eroded_[i], *segmented_eroded);
        pcl::transformPointCloud(*segmented_eroded, *segmented_eroded_trans, cameras_[i]);

        std::stringstream cloud_name;
        cloud_name << "cloud_" << i;
        pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb_handler(cloud_trans);
        vis_->addPointCloud(cloud_trans, rgb_handler, cloud_name.str(), vis_viewpoint_[i * NUM_SUBWINDOWS + 0]);

        if (i>0)
        {
            cloud_name << "_search_pts";
            pcl::PointCloud<PointT>::Ptr cloud_trans_tmp (new pcl::PointCloud<PointT>());
            pcl::transformPointCloud(*transferred_cluster_[i], *cloud_trans_tmp, cameras_[i]);
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> red_source (cloud_trans_tmp, 255, 0, 0);
            vis_->addPointCloud(cloud_trans_tmp, red_source, cloud_name.str(), vis_viewpoint_[i * NUM_SUBWINDOWS + 0]);
        }

        cloud_name << "_supervoxellized";
        vis_->addPointCloud(sv_trans, cloud_name.str(), vis_viewpoint_[i * NUM_SUBWINDOWS + 1]);

        cloud_name << "_nearest_neighbor";
        pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb_handler2(segmented_trans);
        vis_->addPointCloud(segmented_trans, rgb_handler2, cloud_name.str(), vis_viewpoint_[i * NUM_SUBWINDOWS + 2]);

        cloud_name << "_good";
        pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb_handler3(segmented2_trans);
        vis_->addPointCloud(segmented2_trans, rgb_handler3, cloud_name.str(), vis_viewpoint_[i * NUM_SUBWINDOWS + 3]);

        cloud_name << "_region_grown";
        pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb_handler4(segmented3_trans);
        vis_->addPointCloud(segmented3_trans, rgb_handler4, cloud_name.str(), vis_viewpoint_[i * NUM_SUBWINDOWS + 4]);

        cloud_name << "_eroded";
        pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb_handler5(segmented_eroded_trans);
        vis_->addPointCloud(segmented_eroded_trans, rgb_handler5, cloud_name.str(), vis_viewpoint_[i * NUM_SUBWINDOWS + 5]);
    }
    vis_->spin();
}

int
main (int argc, char ** argv)
{
    ros::init (argc, argv, "dynamic_object_learning");

    DOL m;
    m.initialize (argc, argv);

    return 0;
}
