#ifndef DO_LEARNING_H_
#define DO_LEARNING_H_

/*
 * Author: Thomas Faeulhammer
 * Date: June 2015
 *
 * */

#include "ros/ros.h"
#include "do_learning_srv_definitions/learn_object.h"
#include "do_learning_srv_definitions/save_model.h"
#include <pcl/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/search/octree.h>

#include "v4r/KeypointConversions/convertImage.hpp"
#include "v4r/KeypointConversions/convertCloud.hpp"
#include "v4r/KeypointTools/ClusterNormalsToPlanes.hh"
#include "v4r/KeypointTools/DataMatrix2D.hpp"
#include "v4r/KeypointTools/PointTypes.hpp"
#include "v4r/KeypointTools/ZAdaptiveNormals.hh"

#include <pcl_visualizer/remote_pcl_visualizer.h>

struct IndexPoint
{
    int idx;
};

POINT_CLOUD_REGISTER_POINT_STRUCT (IndexPoint,
                                   (int, idx, idx)
                                   )

class DOL
{
private:
    typedef pcl::PointXYZRGB PointT;
    boost::shared_ptr<ros::NodeHandle> n_;
    ros::ServiceServer learn_object_;
    ros::ServiceServer save_model_;
    int normal_method_;
    kp::ClusterNormalsToPlanes::Parameter p_param_;

    pcl::PointCloud<PointT>::Ptr big_cloud_;
    pcl::PointCloud<PointT>::Ptr big_cloud_segmented_;
    std::vector<pcl::PointIndices> obj_indices_eroded_to_original_;
    std::vector<pcl::PointIndices> obj_indices_2_to_filtered_;
    std::vector<pcl::PointIndices> scene_points_;
    std::vector<pcl::PointIndices> transfered_nn_points_;
    std::vector<pcl::PointIndices> transferred_object_indices_without_plane_;
    std::vector<pcl::PointIndices> initial_indices_good_to_unfiltered_;
    std::vector<pcl::PointIndices> obj_indices_3_to_original_;
    std::vector<Eigen::Matrix4f> cameras_;
    std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > keyframes_;
    std::vector< pcl::PointCloud<pcl::Normal>::Ptr > normals_;
    std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > transferred_cluster_;
    std::vector< pcl::PointCloud<pcl::PointXYZRGBA>::Ptr > supervoxeled_clouds_;
    boost::shared_ptr<RemotePCLVisualizer> vis_;
    //boost::shared_ptr<pcl::visualization::PCLVisualizer> vis_;
    std::vector<int> vis_viewpoint_;

    std::vector<size_t> LUT_new2old_indices;

    ///radius to select points in other frames to belong to the same object
    /// bootstraps region growing
    double radius_;
    double eps_angle_;
    double voxel_resolution_;
    double seed_resolution_;
    double ratio_;
    double chop_z_;
    bool visualize_;
    bool do_erosion_;
//    bool do_sift_based_camera_pose_estimation_;
    bool transfer_indices_from_latest_frame_only_;
    pcl::octree::OctreePointCloudSearch<PointT> octree_;

public:

    DOL () : octree_(0.005f)
    {
        radius_ = 0.005f;
        eps_angle_ = 0.99f;
        voxel_resolution_ = 0.005f;
        seed_resolution_ = 0.03f;
        ratio_ = 0.25f;
        chop_z_ = std::numeric_limits<double>::quiet_NaN();
        visualize_ = false;
        do_erosion_ = true;
        transfer_indices_from_latest_frame_only_ = false;
//        do_sift_based_camera_pose_estimation_ = false;

        // Parameters for smooth clustering / plane segmentation
        p_param_.thrAngle=45;
        p_param_.inlDist=0.05;
        p_param_.minPoints=5000;    // minimum number for a plane to be segmented
        p_param_.least_squares_refinement=true;
        p_param_.smooth_clustering=true;
        p_param_.thrAngleSmooth=30;
        p_param_.inlDistSmooth=0.02;
        p_param_.minPointsSmooth=20;    // minimum number for a segment other than a plane

        normal_method_ = 0;

        big_cloud_.reset(new pcl::PointCloud<PointT>);
        big_cloud_segmented_.reset(new pcl::PointCloud<PointT>);
    }

    static Eigen::Matrix4f fromGMTransform(geometry_msgs::Transform & gm_trans)
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
                const pcl::PointCloud<PointT>::Ptr &cloud,
                const pcl::PointCloud<pcl::Normal> &normals_,
                const std::vector<int> &initial,
                std::vector<int> &cluster) const;

    /**
     * @brief transfers object indices from origin into dest camera frame and performs
     *        nearest neighbor search in dest frame.
     * @param origin... id of source frame
     * @param dest... id of destination frame
     * @param nn... nearest neighbors points highlighted (true) in object mask
     */
    void transferIndicesAndNNSearch(size_t origin, size_t dest, std::vector<bool> &object_mask); //std::vector<int> &nn);

    void updatePointNormalsFromSuperVoxels(const pcl::PointCloud<PointT>::Ptr & cloud,
                                           pcl::PointCloud<pcl::Normal>::Ptr & normals_,
                                           const std::vector<int> &object_points,
                                           std::vector<int> & good_neighbours,
                                           pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &supervoxel_cloud);


    void erodeInitialIndices(const pcl::PointCloud<PointT> & cloud,
                             const pcl::PointIndices & initial_indices,
                             pcl::PointIndices & eroded_indices);

    static void createDirIfNotExist(const std::string & dirs)
    {
        boost::filesystem::path dir = dirs;
        if(!boost::filesystem::exists(dir))
        {
            boost::filesystem::create_directory(dir);
        }
    }

    bool save_model (do_learning_srv_definitions::save_model::Request & req,
                  do_learning_srv_definitions::save_model::Response & response);

    bool learn_object (do_learning_srv_definitions::learn_object::Request & req,
                  do_learning_srv_definitions::learn_object::Response & response);

    void initialize (int argc, char ** argv);

    void clearMem()
    {
        keyframes_.clear();
        normals_.clear();
        cameras_.clear();
        transferred_cluster_.clear();
        scene_points_.clear();
        transfered_nn_points_.clear();
        transferred_object_indices_without_plane_.clear();
        initial_indices_good_to_unfiltered_.clear();
        obj_indices_3_to_original_.clear();
        obj_indices_2_to_filtered_.clear();
        obj_indices_eroded_to_original_.clear();
        supervoxeled_clouds_.clear();
    }

    void reserveMem(const size_t &num_elements)
    {
        keyframes_.resize( num_elements );
        normals_.resize(num_elements);
        cameras_.resize( num_elements );
        transferred_cluster_.resize( num_elements );
        scene_points_.resize( num_elements );
        transfered_nn_points_.resize( num_elements );
        transferred_object_indices_without_plane_.resize( num_elements );
        initial_indices_good_to_unfiltered_.resize( num_elements );
        obj_indices_3_to_original_.resize( num_elements );
        obj_indices_2_to_filtered_.resize( num_elements );
        obj_indices_eroded_to_original_.resize( num_elements );
        supervoxeled_clouds_.resize( num_elements );
    }
    static void computeNormals(const pcl::PointCloud<PointT>::ConstPtr & cloud, pcl::PointCloud<pcl::Normal> &normals_, int method = 0);

    /**
     * @brief given a point cloud and a normal cloud, this function computes points belonging to a table
     *  (optional: computes smooth clusters for points not belonging to table)
     * @param cloud
     * @param normals
     * @param p_param
     * @param planes
     */
    static void extractPlanePoints(const pcl::PointCloud<PointT>::ConstPtr &cloud,
                            const pcl::PointCloud<pcl::Normal>::ConstPtr &normals,
                            const kp::ClusterNormalsToPlanes::Parameter p_param,
                            std::vector<kp::ClusterNormalsToPlanes::Plane::Ptr> &planes);

    static void getPlanesNotSupportedByObjectMask(const std::vector<kp::ClusterNormalsToPlanes::Plane::Ptr> &planes,
                                                const pcl::PointIndices object_mask,
                                                std::vector<kp::ClusterNormalsToPlanes::Plane::Ptr> &planes_dst,
                                                pcl::PointIndices &all_plane_indices_wo_object,
                                                float ratio=0.25);
    void visualize();

    /**
     * @brief transforms each keyframe to global coordinate system using given camera
     *  pose and does segmentation based on the computed object indices to create
     *  reconstructed point cloud of scene and object model
     */
    void createBigCloud();


    /**
     * @brief given indices of an image or pointcloud, this function create a boolean mask of the indices
     * @param objectIndices
     * @param image_size
     * @param object_mask (output)
     */
    void createMaskFromIndices( const std::vector<int> &objectIndices,
                                size_t image_size,
                                std::vector<bool> &object_mask);

    /**
     * @brief this function returns the indices of foreground given the foreground mask and considering all true pixels in background mask are being removed
     * @param background_mask (if true, pixel is neglected / not being part of the scene)
     * @param foreground_mask
     * @param new_foreground_indices (output)
     * @param indices of original image which belong to the scene (output)
     */
    void updateIndicesConsideringMask(const std::vector<bool> &background_mask,
                                           const std::vector<bool> &foreground_mask,
                                           std::vector<int> &new_foreground_indices,
                                           std::vector<int> &old_bg_indices);
};


#endif //DO_LEARNING_H_
