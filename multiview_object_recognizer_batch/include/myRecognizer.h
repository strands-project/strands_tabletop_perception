//------------------------ 
//---myRecognizer.h
//
//
//
//------------------------
#ifndef MY_RECOGNIZER_H_
#define MY_RECOGNIZER_H_

#include <iostream>
#include <string>
#include <sstream>
#include <vector>
#include <boost/filesystem.hpp>

#include <v4r/ORFramework/color_ourcvfh_estimator.h>
#include <v4r/ORFramework/global_nn_recognizer_cvfh.h>
#include <v4r/ORFramework/local_recognizer.h>
#include <v4r/ORFramework/metrics.h>
#include <v4r/ORFramework/multi_pipeline_recognizer.h>
#include <v4r/ORFramework/ourcvfh_estimator.h>
#include <v4r/ORFramework/partial_pcd_source.h>
#include <v4r/ORRecognition/ghv.h>

#include <pcl/console/parse.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl/segmentation/planar_polygon_fusion.h>
#include <pcl/segmentation/plane_coefficient_comparator.h>
#include <pcl/segmentation/euclidean_plane_coefficient_comparator.h>
#include <pcl/segmentation/rgb_plane_coefficient_comparator.h>
#include <pcl/segmentation/edge_aware_plane_comparator.h>
#include <pcl/segmentation/euclidean_cluster_comparator.h>
#include <pcl/segmentation/organized_connected_component_segmentation.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

// // #include "v4r/SurfaceSegmenter/Segmenter.h"

namespace bf = boost::filesystem;

typedef faat_pcl::rec_3d_framework::Model<pcl::PointXYZRGB> ModelT;
typedef boost::shared_ptr<ModelT> ModelTPtr;

class myRecognizer
{

private:
    // 	boost::shared_ptr<SIFTHannesRecognizer> sift_local_;
    boost::shared_ptr<faat_pcl::rec_3d_framework::LocalRecognitionPipeline<flann::L1, pcl::PointXYZRGB, pcl::Histogram<128> > > new_sift_local_;
    boost::shared_ptr<faat_pcl::rec_3d_framework::GlobalNNCVFHRecognizer<faat_pcl::Metrics::HistIntersectionUnionDistance, pcl::PointXYZRGB, pcl::Histogram<1327> > > rf_color_ourcvfh_global_;

    typedef pcl::PointXYZRGB PointT;
    boost::shared_ptr< faat_pcl::rec_3d_framework::PartialPCDSource<pcl::PointXYZRGBNormal, pcl::PointXYZRGB> > source;

    boost::shared_ptr<faat_pcl::rec_3d_framework::MultiRecognitionPipeline<PointT> > multi_recog_;

    void doSegmentation (const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZRGB> > & xyz_points,
                         std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> & clusters,
                         std::vector<pcl::PointIndices> & indices, Eigen::Vector4f & table_plane);

    void
    setupRFColorOURCVFH (std::string & model_path,
                         std::string & training_dir);

    float z_dist_;
    bool do_sift_;
    bool do_new_sift_;
    bool do_ourcvfh_;
    bool do_shot_;

    float icp_resolution_;
    int icp_iterations_;

    typedef pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr ConstPointInTPtr;
    typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointInTPtr;

    boost::shared_ptr< std::vector<ModelTPtr> > models_merged_;
    boost::shared_ptr< std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > > transforms_merged_;
    //         segment::Segmenter seg_;
    Eigen::Vector4f table_plane_;
    int segmentation_;
    std::string new_sift_models_, training_input_structure_, training_dir_new_sift_;
    std::string training_dir_shot_;
    pcl::PointCloud<pcl::Normal>::Ptr scene_normals_;

    std::string sift_idx_;
    std::string shot_idx_;

    bool use_table_plane_;
    int CG_SIZE_;
    float CG_THRESHOLD_;
    int sift_knn_;
    int knn_shot_;

public:
    myRecognizer()
    {
        source.reset(new faat_pcl::rec_3d_framework::PartialPCDSource<pcl::PointXYZRGBNormal, pcl::PointXYZRGB>);
        z_dist_ = 1.2f;
        do_sift_ = false;
        do_new_sift_ = true;
        do_ourcvfh_ = true;
        icp_resolution_ = 0.005f;
        icp_iterations_ = 10;
        segmentation_ = 0;
        sift_idx_ = "sift_flann.idx";
        shot_idx_ = "shot_flann.idx";
        use_table_plane_ = true;
        CG_SIZE_ = 3;
        CG_THRESHOLD_ = 0.01f;
        sift_knn_ = 5;
        knn_shot_ = 1;
    }

    void setSiftKnn(int k)
    {
        sift_knn_ = k;
    }

    void setCGParams(int size, float t)
    {
        CG_SIZE_ = size;
        CG_THRESHOLD_ = t;
    }

    void setUseTablePlane(bool b)
    {
        use_table_plane_ = b;
    }

    void setSIFTFlannIdx(std::string & s)
    {
        sift_idx_ = s;
    }

    void setNewSiftParameters(std::string a, std::string b, std::string c)
    {
        new_sift_models_ = a;
        training_input_structure_ = b;
        training_dir_new_sift_ = c;
    }

    void setSHOTParameters(std::string & training_dir)
    {
        training_dir_shot_ = training_dir;
    }

    void setDoShot(bool b)
    {
        do_shot_ = b;
    }

    void setSHOTFlannIdx(std::string & s)
    {
        shot_idx_ = s;
    }

    void
    setDoSift(bool b)
    {
        do_new_sift_ = b;
    }

    void
    setDoOURCVFH(bool b)
    {
        do_ourcvfh_ = b;
    }

    void
    setIcpIterations(int i)
    {
        icp_iterations_ = i;
    }

    void
    setSegmentation(int seg)
    {
        segmentation_ = seg;
    }

    void getTablePlane(Eigen::Vector4f & plane)
    {
        plane = table_plane_;
    }

    void setSceneNormals(pcl::PointCloud<pcl::Normal>::Ptr & normals)
    {
        scene_normals_ = normals;
    }

    static int computeTablePlane (const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZRGB> > & xyz_points, Eigen::Vector4f & table_plane, float z_dist=1.2f);
    void init(std::string models_dir_sift, std::string model_path, std::string training_dir);
    void recognize(const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZRGB> > & xyz_points);
    boost::shared_ptr < std::vector<ModelTPtr> > getModels();
    boost::shared_ptr<std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > > getTransforms();
};

#endif
