#ifndef DO_LEARNING_H_
#define DO_LEARNING_H_

#include "ros/ros.h"
#include "do_learning_srv_definitions/learn_object.h"
#include "do_learning_srv_definitions/save_model.h"
#include <pcl/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/search/octree.h>

#include "v4r/KeypointConversions/convertImage.hpp"
#include "v4r/KeypointConversions/convertCloud.hpp"
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
    int v1, v2;

    std::vector<pcl::PointIndices> object_indices_eroded_;
    std::vector<pcl::PointIndices> object_indices_;
    std::vector<pcl::PointIndices> transferred_object_indices_;
    std::vector<pcl::PointIndices> transferred_object_indices_good_;
    std::vector<Eigen::Matrix4f> cameras_;
    std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > keyframes_;
    std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > transferred_cluster_;
    std::vector< pcl::PointCloud<pcl::PointXYZRGBA>::Ptr > supervoxeled_clouds_;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> vis_;
    std::vector<int> vis_viewpoint_;

    ///radius to select points in other frames to belong to the same object
    /// bootstraps region growing
    double radius_;
    double eps_angle_;
    double voxel_resolution_;
    double seed_resolution_;
    double ratio_;
    bool visualize_;
    bool do_erosion_;
    pcl::octree::OctreePointCloudSearch<PointT> octree;

public:

    DOL () : octree(0.005f)
    {
        radius_ = 0.005f;
        eps_angle_ = 0.99f;
        voxel_resolution_ = 0.005f;
        seed_resolution_ = 0.03f;
        ratio_ = 0.25f;
        visualize_ = false;
        do_erosion_ = true;
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
                const pcl::PointCloud<PointT> &cloud,
                const pcl::PointCloud<pcl::Normal> &normals,
                const pcl::octree::OctreePointCloudSearch<PointT> &tree,
                const std::vector<int> &initial,
                std::vector<int> &cluster);

    void transferIndicesAndNNSearch(size_t origin, size_t dest, std::vector<int> &nn);

    void updatePointNormalsFromSuperVoxels(pcl::PointCloud<PointT>::Ptr & cloud,
                                           pcl::PointCloud<pcl::Normal>::Ptr & normals,
                                           const std::vector<int> & all_neighbours,
                                           std::vector<int> & good_neighbours,
                                           pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &supervoxel_cloud);


    void erodeInitialIndices(const pcl::PointCloud<PointT> & cloud,
                             const std::vector<int> & initial_indices,
                             pcl::PointIndices & eroded_indices);

    static void createDirIfNotExist(std::string & dirs)
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
        cameras_.clear();
        transferred_cluster_.clear();
        transferred_object_indices_.clear();
        transferred_object_indices_good_.clear();
        object_indices_.clear();
        object_indices_eroded_.clear();
        supervoxeled_clouds_.clear();
    }

    void reserveMem(const size_t &num_elements)
    {
        keyframes_.resize( num_elements );
        cameras_.resize( num_elements );
        transferred_cluster_.resize( num_elements );
        transferred_object_indices_.resize( num_elements );
        transferred_object_indices_good_.resize( num_elements );
        object_indices_.resize( num_elements );
        object_indices_eroded_.resize( num_elements );
        supervoxeled_clouds_.resize( num_elements );
    }

    void visualize();
};


#endif //DO_LEARNING_H_
