/*
 * do_learning.cpp
 *
 * incrementally learning of objects by
 * transfering object indices from initial cloud to the remaining clouds
 * using given camera poses
 *
 *  Created on: June, 2015
 *      Author: Aitor Aldoma, Thomas Faeulhammer
 */

#define EIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET

#include "do_learning.h"

#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float32.h>
#include <stdlib.h>
#include <thread>

#include <pcl_conversions.h>
#include <pcl/common/angles.h>
#include <pcl/common/time.h>
#include <pcl/common/transforms.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/io/pcd_io.h>
#include <pcl/segmentation/supervoxel_clustering.h>

#include <v4r/KeypointConversions/convertNormals.hpp>
#include <v4r/KeypointTools/invPose.hpp>
#include <v4r/KeypointTools/PointTypes.hpp>
#include <v4r/KeypointTools/PoseIO.hpp>
#include <v4r/KeypointTools/ScopeTime.hpp>
#include <v4r/KeypointTools/toString.hpp>
#include <v4r/KeypointSlam/KeypointSlamRGBD2.hh>
#include <v4r/KeypointSlam/ProjBundleAdjuster.hh>
#include <v4r/ORUtils/noise_model_based_cloud_integration.h>
#include <v4r/ORUtils/noise_models.h>
#include <v4r/ORUtils/pcl_visualization_utils.h>
#include <v4r/utils/filesystem_utils.h>

#define NUM_SUBWINDOWS 7

void DOL::extractEuclideanClustersSmooth (
        const pcl::PointCloud<PointT>::Ptr &cloud,
        const pcl::PointCloud<pcl::Normal> &normals,
        const std::vector<int> &initial,
        std::vector<int> &cluster)
{
    float tolerance = radius_;

    if (cloud->points.size () != normals.points.size ())
    {
        PCL_ERROR ("[pcl::extractEuclideanClusters] Number of points in the input point cloud (%zu) different than normals (%zu)!\n", cloud->points.size (), normals.points.size ());
        return;
    }

    pcl::octree::OctreePointCloudSearch<PointT> octree(0.005f);
    octree.setInputCloud ( cloud );
    octree.addPointsFromInputCloud ();

    cluster.clear(); // is this okay???

    // Create a bool vector of processed point indices, and initialize it to false
    std::vector<bool> to_grow (cloud->points.size (), false);
    std::vector<bool> in_cluster (cloud->points.size (), false);

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
        for (size_t i = 0; i < cloud->points.size (); i++)
        {
            if (!to_grow[i])
                continue;

            to_grow[i] = false;

            if (octree.radiusSearch (cloud->points[i], tolerance, nn_indices, nn_distances))
            {
                for (size_t j = 0; j < nn_indices.size (); j++) // is nn_indices[0] the same point?
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
        for (size_t i = 0; i < cloud->points.size (); ++i)
        {
            if(to_grow[i])
                ngrow++;
        }

        if(ngrow == 0)
            stop = true;
    }

    for (size_t i = 0; i < cloud->points.size (); ++i)
    {
        if(in_cluster[i])
            cluster.push_back(static_cast<int>(i));
    }
}

void DOL::updatePointNormalsFromSuperVoxels(const pcl::PointCloud<PointT>::Ptr & cloud,
                                            pcl::PointCloud<pcl::Normal>::Ptr & normals,
                                            const std::vector<int> & obj_points,
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

    for(size_t id = 0; id < obj_points.size(); id++)
    {
        int sv_idx = label_to_idx[ supervoxels_labels_cloud->at( obj_points[ id ] ).label];
        if(sv_idx >= 0 && sv_idx < sv_size)
        {
            label_count_nn[sv_idx]++;
        }
    }

    for(size_t id = 0; id < obj_points.size(); id++)
    {
        int sv_idx = label_to_idx[ supervoxels_labels_cloud->at( obj_points[ id ] ).label];
        if(sv_idx < 0 || sv_idx >= sv_size)
            continue;

        if( (label_count_nn[sv_idx] / (float)label_count[sv_idx]) > ratio_)
        {
            good_neighbours.push_back( obj_points[ id ] );
        }
    }
}

void DOL::transferIndicesAndNNSearch(size_t origin, size_t dest, std::vector<int> &nn)
{
    pcl::PointCloud<PointT>::Ptr cloud_origin_filtered(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr segmented(new pcl::PointCloud<PointT>);

    pcl::copyPointCloud(*keyframes_[origin], scene_points_[origin], *cloud_origin_filtered);

    if (do_erosion_)
    {
        pcl::copyPointCloud(*cloud_origin_filtered, obj_indices_eroded_to_original_[origin], *segmented);
    }
    else
    {
        pcl::copyPointCloud(*cloud_origin_filtered, obj_indices_2_to_filtered_[origin], *segmented);
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
        if (octree_.radiusSearch (segmented_trans->points[i], radius_, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
        {
            all_neighbours.insert(pointIdxRadiusSearch.begin(), pointIdxRadiusSearch.end());
        }
    }

    std::copy(all_neighbours.begin(), all_neighbours.end(), std::back_inserter(nn));

    std::cout << "Found nearest neighbor: " << nn.size() << std::endl;
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
    std::vector<std::vector<int> > indices;

    weights.resize(keyframes_.size());
    indices.resize(keyframes_.size());
    object_indices_clouds.resize(keyframes_.size());

    for(size_t i=0; i < keyframes_.size(); i++)
    {
        indices[i] = obj_indices_eroded_to_original_[i].indices;

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
        nm.setInputNormals(normals_[i]);
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
    nmIntegration.setInputNormals(normals_);
    nmIntegration.setMinPointsPerVoxel(1);
    nmIntegration.setFinalResolution(0.002f);
    nmIntegration.setIndices(indices);
    nmIntegration.setThresholdSameSurface(0.01f);
    nmIntegration.compute(octree_cloud);

    pcl::PointCloud<pcl::Normal>::Ptr octree_normals;
    nmIntegration.getOutputNormals(octree_normals);

    std::stringstream export_to_rs;
    export_to_rs << recognition_structure_dir << "/" << model_name << "/";
    std::string export_to = export_to_rs.str();

    createDirIfNotExist(recognition_structure_dir);
    createDirIfNotExist(models_dir);
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

void DOL::computeNormals(const pcl::PointCloud<PointT>::ConstPtr &cloud, pcl::PointCloud<pcl::Normal> &normals)
{
    if(normal_method_== 0)
    {
        pcl::NormalEstimation<PointT, pcl::Normal> n3d;
        n3d.setRadiusSearch (0.01f);
        n3d.setInputCloud (cloud);
        n3d.compute (normals);
    }
    else if(normal_method_ == 1)
    {
        pcl::IntegralImageNormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
        ne.setNormalEstimationMethod (ne.AVERAGE_3D_GRADIENT);
        ne.setMaxDepthChangeFactor(0.02f);
        ne.setNormalSmoothingSize(15.f);//20.0f);
        ne.setDepthDependentSmoothing(false);//param.normals_depth_dependent_smoothing);
        ne.setInputCloud(cloud);
        ne.setViewPoint(0,0,0);
        ne.compute(normals);
    }
    else //if(normal_method_ == 2)
    {
        kp::DataMatrix2D<Eigen::Vector3f>::Ptr kp_cloud( new kp::DataMatrix2D<Eigen::Vector3f>() );
        kp::DataMatrix2D<Eigen::Vector3f>::Ptr kp_normals_tmp( new kp::DataMatrix2D<Eigen::Vector3f>() );
        kp::convertCloud(*cloud, *kp_cloud);
        nest_->compute(*kp_cloud, *kp_normals_tmp);
        kp::convertNormals(*kp_normals_tmp, normals);
    }

    // Normalize normals to unit length
    for ( size_t normal_pt_id = 0; normal_pt_id < normals.points.size(); normal_pt_id++)
    {
        Eigen::Vector3f n1 = normals.points[normal_pt_id].getNormalVector3fMap();
        n1.normalize();
        normals.points[normal_pt_id].normal_x = n1(0);
        normals.points[normal_pt_id].normal_y = n1(1);
        normals.points[normal_pt_id].normal_z = n1(2);
    }
}

void DOL::extractPlanePoints(const pcl::PointCloud<PointT>::ConstPtr &cloud,
                             const pcl::PointCloud<pcl::Normal>::ConstPtr &normals,
                             std::vector<kp::ClusterNormalsToPlanes::Plane::Ptr> &planes)
{
    kp::DataMatrix2D<Eigen::Vector3f>::Ptr kp_cloud( new kp::DataMatrix2D<Eigen::Vector3f>() );
    kp::DataMatrix2D<Eigen::Vector3f>::Ptr kp_normals( new kp::DataMatrix2D<Eigen::Vector3f>() );
    kp::convertCloud(*cloud, *kp_cloud);
    kp::convertNormals(*normals, *kp_normals);
    pest_->compute(*kp_cloud, *kp_normals, planes);
}

void DOL::getPlanesNotSupportedByObjectMask(const std::vector<kp::ClusterNormalsToPlanes::Plane::Ptr> &planes,
                                            const pcl::PointIndices object_mask,
                                            std::vector<kp::ClusterNormalsToPlanes::Plane::Ptr> &planes_dst,
                                            pcl::PointIndices &all_plane_indices_wo_object,
                                            float ratio)
{
    planes_dst.clear();
    all_plane_indices_wo_object.indices.clear();

    for(size_t cluster_id=0; cluster_id<planes.size(); cluster_id++)
    {
        size_t num_obj_pts = 0;

        if (planes[cluster_id]->is_plane)
        {
            for (size_t cluster_pt_id=0; cluster_pt_id<planes[cluster_id]->indices.size(); cluster_pt_id++)
            {
                for (size_t obj_pt_id=0; obj_pt_id<object_mask.indices.size(); obj_pt_id++)
                {
                    if (object_mask.indices[obj_pt_id] == planes[cluster_id]->indices[cluster_pt_id])
                    {
                        num_obj_pts++;
                    }
                }
            }

            if ( num_obj_pts < ratio * planes[cluster_id]->indices.size() )
            {
                planes_dst.push_back( planes[cluster_id] );
                all_plane_indices_wo_object.indices.insert(all_plane_indices_wo_object.indices.end(),
                                                           planes[cluster_id]->indices.begin(),
                                                           planes[cluster_id]->indices.end());
            }

        }
    }
}

void DOL::extractObjectIndicesWithoutPlane(const pcl::PointIndices &inputIndices,
                                           const std::vector<kp::ClusterNormalsToPlanes::Plane::Ptr> &planes,
                                           pcl::PointIndices &outputIndices)
{
    std::vector<size_t> indices_to_delete;
    indices_to_delete.resize(inputIndices.indices.size());
    size_t kept = 0;

    for (size_t cluster_id=0; cluster_id < planes.size(); cluster_id++)
    {
        if (planes[cluster_id]->is_plane)
        {
            std::vector<int>::const_iterator cluster_pt_it;
            for ( cluster_pt_it = planes[ cluster_id ]->indices.begin(); cluster_pt_it != planes[ cluster_id ]->indices.end(); ++cluster_pt_it)
            {
                for (size_t obj_pt_id = 0; obj_pt_id < inputIndices.indices.size(); obj_pt_id++)
                {
                    if ( *cluster_pt_it == inputIndices.indices[ obj_pt_id ] )
                    {
                        indices_to_delete[kept] = obj_pt_id;
                        kept++;
                    }
                }
            }
        }
    }
    indices_to_delete.resize(kept);
    outputIndices.indices = erase_indices<int>(inputIndices.indices, indices_to_delete);
}

void DOL::createMaskFromIndices( const std::vector<int> &objectIndices,
                            size_t image_size,
                            std::vector<bool> &mask)
{
    mask.resize( image_size );

    for (size_t pt_id=0; pt_id < image_size; pt_id++)
    {
        mask [pt_id] = false;
    }

    for (size_t obj_pt_id = 0; obj_pt_id < objectIndices.size(); obj_pt_id++)
    {
        mask [objectIndices[obj_pt_id]] = true;
    }
}

void DOL::updateIndicesConsideringMask(const std::vector<bool> &bg_mask,
                                       const std::vector<bool> &fg_mask,
                                       std::vector<int> &fg_indices,
                                       std::vector<int> &old_bg_indices)
{
    fg_indices.resize( fg_mask.size() );
    old_bg_indices.resize( fg_mask.size() );

    size_t kept=0;
    size_t num_new_pxs = 0;
    for(size_t old_px_id=0; old_px_id < bg_mask.size(); old_px_id++)
    {
        if( !bg_mask[old_px_id] && fg_mask[old_px_id] ) // pixel is not neglected and belongs to object -> save
        {
            fg_indices[kept] = num_new_pxs;
            kept++;
        }
        if( !bg_mask [old_px_id] )
        {
            old_bg_indices[num_new_pxs] = old_px_id;
            num_new_pxs++;
        }
    }
    fg_indices.resize(kept);
    old_bg_indices.resize(num_new_pxs);
}

bool
DOL::learn_object (do_learning_srv_definitions::learn_object::Request & req,
                   do_learning_srv_definitions::learn_object::Response & response)
{
    assert(req.transforms.size() == req.keyframes.size());
    clearMem();
    reserveMem(req.keyframes.size());

    //TODO: If the indices are empty, probably means that the point of view does not contain the object, propagate last non-empy indices
    for(size_t i=0; i < keyframes_.size(); i++)
    {
        std::cout << "Computing indices for cloud " << i << std::endl
                  << "===================================" << std::endl;

        pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
        pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>());
        pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
        pcl::PointCloud<pcl::Normal>::Ptr normals_filtered (new pcl::PointCloud<pcl::Normal>());
        std::vector<kp::ClusterNormalsToPlanes::Plane::Ptr> planes, planes_wo_obj;
        pcl::PointIndicesPtr obj_indices_updated (new pcl::PointIndices);
        pcl::PointIndices plane_and_nan_points;
        std::vector<bool> pixel_is_object;
        std::vector<bool> pixel_is_neglected;

        keyframes_[i].reset(new pcl::PointCloud<PointT>());
        normals_[i].reset(new pcl::PointCloud<pcl::Normal>());
        pcl::fromROSMsg(req.keyframes[i], *keyframes_[i]);
        cameras_[i] = fromGMTransform(req.transforms[i]);

        computeNormals(keyframes_[i], *normals_[i]);
        assert(keyframes_[i]->points.size() == normals_[i]->points.size());

        octree_.setInputCloud ( keyframes_[i] );
        octree_.addPointsFromInputCloud ();

        if (i==0)   // for first frame use given initial indices and erode them
        {
            for(size_t i=0; i < req.intial_object_indices.size(); i++)
            {
                transfered_nn_points_[0].indices.push_back(req.intial_object_indices[i].data);
            }
            //erode mask
            erodeInitialIndices(*keyframes_[0], transfered_nn_points_[0], obj_indices_eroded_to_original_[0]);
            obj_indices_2_to_filtered_[0] = transfered_nn_points_[0];
        }
        else
        {
            transferIndicesAndNNSearch(i-1, i, transfered_nn_points_[i].indices);
        }


        createMaskFromIndices(transfered_nn_points_[i].indices, keyframes_[i]->points.size(), pixel_is_object);
        extractPlanePoints(keyframes_[i], normals_[i], planes);
        getPlanesNotSupportedByObjectMask(planes,
                                          transfered_nn_points_[i],
                                          planes_wo_obj,
                                          plane_and_nan_points);

        // append indices with nan values or points further away than chop_z_ parameter
        std::vector<int> nan_indices;
        nan_indices.resize( keyframes_[i]->points.size() );
        size_t kept=0;
        for(size_t pt_id = 0; pt_id<keyframes_[i]->points.size(); pt_id++)
        {
            if ( !pcl::isFinite(keyframes_[i]->points[pt_id]) || keyframes_[i]->points[pt_id].z > chop_z_)
            {
                nan_indices[kept] = pt_id;
                kept++;
            }
        }
        nan_indices.resize(kept);
        plane_and_nan_points.indices.insert( plane_and_nan_points.indices.end(),
                                                    nan_indices.begin(), nan_indices.end() );

        createMaskFromIndices(plane_and_nan_points.indices, keyframes_[i]->points.size(), pixel_is_neglected);

        updateIndicesConsideringMask(pixel_is_neglected, pixel_is_object, obj_indices_updated->indices, scene_points_[i].indices);

        pcl::copyPointCloud(*keyframes_[i], scene_points_[i], *cloud_filtered);
        pcl::copyPointCloud(*normals_[i], scene_points_[i], *normals_filtered);


        //#define DEBUG_SEGMENTATION
#ifdef DEBUG_SEGMENTATION
        {
            pcl::visualization::PCLVisualizer vis("segmented cloud");
            for(size_t cluster_id=0; cluster_id<planes.size(); cluster_id++)
            {
                vis.removeAllPointCloud();
                pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb_handler(keyframes_[i]);
                vis.addPointCloud(keyframes_[i], rgb_handler, "original_cloud");


                pcl::PointCloud<PointT>::Ptr segmented (new pcl::PointCloud<PointT>());
                pcl::copyPointCloud(*keyframes_[i], planes[cluster_id]->indices, *segmented);
                if (planes[cluster_id]->is_plane)
                {
                    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> red_source (segmented, 255, 0, 0);
                    vis.addPointCloud(segmented, red_source, "segmented");
                }
                else
                {
                    break;
                    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> green_source (segmented, 0, 255, 0);
                    vis.addPointCloud(segmented, green_source, "segmented");
                }
                vis.spin();
            }
            vis.removeAllPointClouds();
            pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb_handler(keyframes_[i]);
            vis.addPointCloud(keyframes_[i], rgb_handler, "original_cloud");
            vis.spin();
        }
#endif

        updatePointNormalsFromSuperVoxels(cloud_filtered,
                                          normals_filtered,
                                          obj_indices_updated->indices,
                                          initial_indices_good_to_unfiltered_[i].indices,
                                          supervoxeled_clouds_[i]);

        extractEuclideanClustersSmooth(cloud_filtered, *normals_filtered,
                                       initial_indices_good_to_unfiltered_[i].indices,
                                       obj_indices_2_to_filtered_[i].indices);


        // Transferring indices corresponding to filtered cloud such that they correspond to original (unfiltered) cloud.
        obj_indices_3_to_original_[i].indices.resize( obj_indices_2_to_filtered_[i].indices.size() );
        for (size_t obj_pt_id=0; obj_pt_id<obj_indices_2_to_filtered_[i].indices.size(); obj_pt_id++)
        {
            obj_indices_3_to_original_[i].indices[obj_pt_id] = scene_points_[i].indices[ obj_indices_2_to_filtered_[i].indices[obj_pt_id] ];
        }

        erodeInitialIndices(*keyframes_[i], obj_indices_3_to_original_[i], obj_indices_eroded_to_original_[i]);

        std::cout << "Found " << transfered_nn_points_[i].indices.size() << " nearest neighbors before growing." << std::endl
                  << "After updatePointNormalsFromSuperVoxels size: " << initial_indices_good_to_unfiltered_[i].indices.size() << std::endl
                  << "size of final cluster: " << obj_indices_2_to_filtered_[i].indices.size() << std::endl
                  << "size of final cluster after erosion: " << obj_indices_eroded_to_original_[i].indices.size() << std::endl << std::endl;
    }

    createBigCloud();

    if(visualize_)
    {
        visualize();
    }

    return true;
}

void DOL::createBigCloud()
{
    for(size_t i=0; i < keyframes_.size(); i++)
    {
        pcl::PointCloud<PointT>::Ptr cloud_trans (new pcl::PointCloud<PointT>());
        pcl::PointCloud<PointT>::Ptr cloud_trans_filtered (new pcl::PointCloud<PointT>());
        pcl::transformPointCloud(*keyframes_[i], *cloud_trans, cameras_[i]);
        pcl::copyPointCloud(*cloud_trans, scene_points_[i], *cloud_trans_filtered);
        *big_cloud_ += *cloud_trans_filtered;

        pcl::PointCloud<PointT>::Ptr segmented_trans (new pcl::PointCloud<PointT>());
        pcl::copyPointCloud(*cloud_trans_filtered, obj_indices_2_to_filtered_[i], *segmented_trans);
        *big_cloud_segmented_ += *segmented_trans;
    }
}

void DOL::visualize()
{
    pcl::visualization::PCLVisualizer vis("segmented cloud");
    int v1, v2;
    vis.createViewPort(0,0,0.5,1,v1);
    vis.createViewPort(0.5,0,1,1,v2);
    vis.addPointCloud(big_cloud_, "big", v1);
    vis.addPointCloud(big_cloud_segmented_, "segmented", v2);
    vis.spinOnce();

    if (!vis_) {
        vis_.reset(new pcl::visualization::PCLVisualizer());
        std::vector<std::string> subwindow_title;
        subwindow_title.push_back("original scene");
        subwindow_title.push_back("filtered scene");
        subwindow_title.push_back("supervoxelled scene");
        subwindow_title.push_back("after nearest neighbor search");
        subwindow_title.push_back("good points");
        subwindow_title.push_back("before 2D erosion");
        subwindow_title.push_back("after 2D erosion");
        vis_viewpoint_ = faat_pcl::utils::visualization_framework (vis_, keyframes_.size(), NUM_SUBWINDOWS, subwindow_title);
    }

    for(size_t i=0; i < keyframes_.size(); i++)
    {
        pcl::PointCloud<PointT>::Ptr cloud_trans (new pcl::PointCloud<PointT>());
        pcl::PointCloud<PointT>::Ptr cloud_trans_filtered (new pcl::PointCloud<PointT>());
        pcl::transformPointCloud(*keyframes_[i], *cloud_trans, cameras_[i]);
        pcl::copyPointCloud(*cloud_trans, scene_points_[i], *cloud_trans_filtered);

        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr sv_trans (new pcl::PointCloud<pcl::PointXYZRGBA>());
        pcl::transformPointCloud(*supervoxeled_clouds_[i], *sv_trans, cameras_[i]);

        pcl::PointCloud<PointT>::Ptr segmented (new pcl::PointCloud<PointT>());
        pcl::PointCloud<PointT>::Ptr segmented_trans (new pcl::PointCloud<PointT>());
        pcl::copyPointCloud(*keyframes_[i], transfered_nn_points_[i], *segmented);
        pcl::transformPointCloud(*segmented, *segmented_trans, cameras_[i]);

        pcl::PointCloud<PointT>::Ptr segmented2_trans (new pcl::PointCloud<PointT>());
        pcl::copyPointCloud(*cloud_trans_filtered, initial_indices_good_to_unfiltered_[i], *segmented2_trans);

        pcl::PointCloud<PointT>::Ptr segmented3_trans (new pcl::PointCloud<PointT>());
        pcl::copyPointCloud(*cloud_trans_filtered, obj_indices_2_to_filtered_[i], *segmented3_trans);

        pcl::PointCloud<PointT>::Ptr segmented_eroded_trans (new pcl::PointCloud<PointT>());
        pcl::copyPointCloud(*cloud_trans, obj_indices_eroded_to_original_[i], *segmented_eroded_trans);

        std::stringstream cloud_name;
        cloud_name << "cloud_" << i;
        pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb_handler0(cloud_trans);
        vis_->addPointCloud(cloud_trans, rgb_handler0, cloud_name.str(), vis_viewpoint_[i * NUM_SUBWINDOWS + 0]);

        if (i>0)
        {
            cloud_name << "_search_pts";
            pcl::PointCloud<PointT>::Ptr cloud_trans_tmp (new pcl::PointCloud<PointT>());
            pcl::transformPointCloud(*transferred_cluster_[i], *cloud_trans_tmp, cameras_[i]);
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> red_source (cloud_trans_tmp, 255, 0, 0);
            vis_->addPointCloud(cloud_trans_tmp, red_source, cloud_name.str(), vis_viewpoint_[i * NUM_SUBWINDOWS + 0]);
        }

        cloud_name << "_filtered";
        pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb_handler1(cloud_trans_filtered);
        vis_->addPointCloud(cloud_trans_filtered, rgb_handler1, cloud_name.str(), vis_viewpoint_[i * NUM_SUBWINDOWS + 1]);

        cloud_name << "_supervoxellized";
        vis_->addPointCloud(sv_trans, cloud_name.str(), vis_viewpoint_[i * NUM_SUBWINDOWS + 2]);

        cloud_name << "_nearest_neighbor";
        pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb_handler2(segmented_trans);
        vis_->addPointCloud(segmented_trans, rgb_handler2, cloud_name.str(), vis_viewpoint_[i * NUM_SUBWINDOWS + 3]);

        cloud_name << "_good";
        pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb_handler3(segmented2_trans);
        vis_->addPointCloud(segmented2_trans, rgb_handler3, cloud_name.str(), vis_viewpoint_[i * NUM_SUBWINDOWS + 4]);

        cloud_name << "_region_grown";
        pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb_handler4(segmented3_trans);
        vis_->addPointCloud(segmented3_trans, rgb_handler4, cloud_name.str(), vis_viewpoint_[i * NUM_SUBWINDOWS + 5]);

        cloud_name << "_eroded";
        pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb_handler5(segmented_eroded_trans);
        vis_->addPointCloud(segmented_eroded_trans, rgb_handler5, cloud_name.str(), vis_viewpoint_[i * NUM_SUBWINDOWS + 6]);
    }
    vis_->spin();
}


void DOL::initialize (int argc, char ** argv)
{
    double inlDist, minPoints;

    n_.reset( new ros::NodeHandle ( "~" ) );
    n_->getParam ( "radius", radius_);
    n_->getParam ( "dot_product", eps_angle_);
    n_->getParam ( "seed_res", seed_resolution_);
    n_->getParam ( "voxel_res", voxel_resolution_);
    n_->getParam ( "ratio", ratio_);
    n_->getParam ( "visualize", visualize_);
    n_->getParam ( "do_erosion", do_erosion_);
    n_->getParam ( "chop_z", chop_z_);

    if( n_->getParam ( "inlier_threshold_plane_seg", inlDist) )
        p_param_.inlDist = static_cast<float> (inlDist);

    if ( n_->getParam ( "min_plane_points", minPoints) )
        p_param_.minPoints = static_cast<float> (minPoints);

    learn_object_  = n_->advertiseService ("learn_object", &DOL::learn_object, this);
    save_model_  = n_->advertiseService ("save_model", &DOL::save_model, this);

    pest_.reset(new kp::ClusterNormalsToPlanes(p_param_));

    std::cout << "Started dynamic object learning with parameters: " << std::endl
              << "===================================================" << std::endl
              << "radius: " << radius_ << std::endl
              << "eps_angle: " << eps_angle_ << std::endl
              << "voxel_resolution: " << seed_resolution_ << std::endl
              << "ratio: " << ratio_ << std::endl
              << "===================================================" << std::endl << std::endl;

    ros::spin ();
}

int
main (int argc, char ** argv)
{
    ros::init (argc, argv, "dynamic_object_learning");
    DOL m;
    m.initialize (argc, argv);

    return 0;
}
