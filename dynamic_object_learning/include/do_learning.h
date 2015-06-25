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
#include "v4r/KeypointTools/ClusterNormalsToPlanes.hh"
#include "v4r/KeypointTools/DataMatrix2D.hpp"
#include "v4r/KeypointTools/PointTypes.hpp"
#include "v4r/KeypointTools/ZAdaptiveNormals.hh"
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
    int normal_method_;

    std::vector<pcl::PointIndices> object_indices_eroded_;
    std::vector<pcl::PointIndices> object_indices_;
    std::vector<pcl::PointIndices> transferred_object_indices_;
    std::vector<pcl::PointIndices> transferred_object_indices_without_plane_;
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

    kp::ClusterNormalsToPlanes::Ptr pest_;
    kp::ClusterNormalsToPlanes::Parameter p_param_;
    kp::ZAdaptiveNormals::Ptr nest_;
    kp::ZAdaptiveNormals::Parameter n_param_;

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

        // Parameters for smooth clustering / plane segmentation
        p_param_.thrAngle=45;
        p_param_.inlDist=0.05;
        p_param_.minPoints=5000;    // minimum number for a plane to be segmented
        p_param_.least_squares_refinement=true;
        p_param_.smooth_clustering=true;
        p_param_.thrAngleSmooth=30;
        p_param_.inlDistSmooth=0.02;
        p_param_.minPointsSmooth=20;    // minimum number for a segment other than a plane

        n_param_.adaptive = true;
        nest_.reset(new kp::ZAdaptiveNormals(n_param_));

        normal_method_ = 0;
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
                             const pcl::PointIndices & initial_indices,
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
        transferred_object_indices_without_plane_.clear();
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
        transferred_object_indices_without_plane_.resize( num_elements );
        transferred_object_indices_good_.resize( num_elements );
        object_indices_.resize( num_elements );
        object_indices_eroded_.resize( num_elements );
        supervoxeled_clouds_.resize( num_elements );
    }
    void computeNormals(const pcl::PointCloud<PointT>::ConstPtr & cloud, pcl::PointCloud<pcl::Normal> &normals);
    void extractPlanePoints(const pcl::PointCloud<PointT>::ConstPtr &cloud, const pcl::PointCloud<pcl::Normal>::ConstPtr &normals, std::vector<kp::ClusterNormalsToPlanes::Plane::Ptr> &planes);
    void getPlanesNotSupportedByObjectMask(const std::vector<kp::ClusterNormalsToPlanes::Plane::Ptr> &planes,
                                                const pcl::PointIndices object_mask,
                                                std::vector<kp::ClusterNormalsToPlanes::Plane::Ptr> &planes_dst,
                                                pcl::PointIndices &all_plane_indices_wo_object,
                                                float ratio=0.25);

    void extractObjectIndicesWithoutPlane(const pcl::PointIndices &inputIndices,
                                          const std::vector<kp::ClusterNormalsToPlanes::Plane::Ptr> &planes,
                                          pcl::PointIndices &outputIndices);
    void visualize();

    template<typename T>
    inline std::vector<T> erase_indices(const std::vector<T>& data, std::vector<size_t>& indicesToDelete/* can't assume copy elision, don't pass-by-value */)
    {
        if(indicesToDelete.empty())
            return data;

        std::vector<T> ret;
        ret.reserve(data.size() - indicesToDelete.size());

        std::sort(indicesToDelete.begin(), indicesToDelete.end());

        // new we can assume there is at least 1 element to delete. copy blocks at a time.
        typename std::vector<T>::const_iterator itBlockBegin = data.begin();
        for(std::vector<size_t>::const_iterator it = indicesToDelete.begin(); it != indicesToDelete.end(); ++ it)
        {
            typename std::vector<T>::const_iterator itBlockEnd = data.begin() + *it;
            if(itBlockBegin != itBlockEnd)
            {
                std::copy(itBlockBegin, itBlockEnd, std::back_inserter(ret));
            }
            itBlockBegin = itBlockEnd + 1;
        }

        // copy last block.
        if(itBlockBegin != data.end())
        {
            std::copy(itBlockBegin, data.end(), std::back_inserter(ret));
        }

        return ret;
    }
};


#endif //DO_LEARNING_H_
