#ifndef MYGRAPHCLASSES_H
#define MYGRAPHCLASSES_H

#include <vector>
#include <iostream>
#include <string>
#include <sstream>
#include <opencv2/core/core.hpp>
#include <opencv2/nonfree/features2d.hpp>
//#include <pcl/common/common.h>

#include <pcl/features/normal_3d.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/pfh.h>
#include <pcl/features/vfh.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/registration/correspondence_types.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/search/impl/flann_search.hpp>
#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl/segmentation/planar_polygon_fusion.h>
#include <pcl/segmentation/plane_coefficient_comparator.h>
#include <pcl/segmentation/euclidean_plane_coefficient_comparator.h>
#include <pcl/segmentation/rgb_plane_coefficient_comparator.h>
#include <pcl/segmentation/edge_aware_plane_comparator.h>
#include <pcl/segmentation/euclidean_cluster_comparator.h>
#include <pcl/segmentation/organized_connected_component_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <faat_pcl/3d_rec_framework/defines/faat_3d_rec_framework_defines.h>
#include <faat_pcl/3d_rec_framework/feature_wrapper/local/image/sift_local_estimator.h>
#include <faat_pcl/3d_rec_framework/pc_source/model_only_source.h>
#include <faat_pcl/3d_rec_framework/pc_source/partial_pcd_source.h>
#include <faat_pcl/3d_rec_framework/pipeline/multi_pipeline_recognizer.h>
#include <faat_pcl/3d_rec_framework/segmentation/multiplane_segmentation.h>
#include <faat_pcl/recognition/hv/hv_go.h>
#include <faat_pcl/recognition/hv/hv_go_1.h>
#include <faat_pcl/recognition/hv/hv_go_3D.h>
#include <faat_pcl/registration/fast_icp_with_gc.h>
#include <faat_pcl/registration/mv_lm_icp.h>
#include <faat_pcl/registration/registration_utils.h>
// #include <faat_pcl/utils/filesystem_utils.h>
#include <faat_pcl/utils/miscellaneous.h>
#include <faat_pcl/utils/noise_models.h>
#include <faat_pcl/utils/noise_model_based_cloud_integration.h>
#include <faat_pcl/utils/pcl_opencv.h>
#include <faat_pcl/utils/registration_utils.h>

#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
#include <pcl_conversions.h>
#include "boost_graph_extension.h"
// #include "functions.h"
//#include "visual_recognizer/Hypotheses.h"
#include "recognition_srv_definitions/recognize.h"
#include "recognition_srv_definitions/multiview_recognize.h"
#include "segmenter.h"

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointInT;
typedef PointInT::ConstPtr ConstPointInTPtr;
typedef boost::shared_ptr< PointInT > PointInTPtr;
typedef pcl::PointXYZRGB PointT;
typedef pcl::Histogram<128> FeatureT;
typedef flann::L1<float> DistT;

typedef faat_pcl::rec_3d_framework::Model<PointT> ModelT;
typedef boost::shared_ptr<ModelT> ModelTPtr;
typedef typename pcl::PointCloud<PointT>::ConstPtr ConstPointInTPtr;

class multiviewGraph
{
private:
    Graph grph_, grph_final_;
    std::vector<Edge> edges_, best_edges_;
    std::string models_dir_;
    std::string scene_name_;
    boost::shared_ptr < faat_pcl::rec_3d_framework::ModelOnlySource<pcl::PointXYZRGBNormal, PointT> > models_source_;
    pcl::visualization::PCLVisualizer::Ptr edge_vis;
    bool visualize_output_;
    bool go_3d_;
    int icp_iter_;
    int opt_type_;
    std::string gt_or_ouput_dir_;
    double chop_at_z_;
    float icp_resolution_;
    float icp_max_correspondence_distance_;
    bool do_reverse_hyp_extension;
    pcl::visualization::PCLVisualizer::Ptr vis_;
    bool scene_to_scene_;
    bool use_robot_pose_;
    bool use_gc_s2s_;

    bool use_unverified_single_view_hypotheses;

    //GO3D parameters
    float go3d_color_sigma_;
    float go3d_outlier_regularizer_;
    float go3d_clutter_regularizer_;
    float go3d_clutter_radius_;
    float go3d_inlier_threshold_;

    bool go3d_detect_clutter_;
    bool go3d_add_planes_;
    bool go3d_icp_;
    bool go3d_icp_model_to_scene_;
    bool go3d_use_supervoxels_;

    float go3d_and_icp_resolution_;


    //Noise model parameters
    float max_angle_;
    float lateral_sigma_;
    float nm_integration_min_weight_;

    //Multiview refinement parameters
    bool mv_icp_;
    float max_keypoint_dist_mv_;
    int mv_iterations_;
    float min_overlap_mv_;
    int mv_keypoints_;
    float inlier_threshold_ ;
    float max_corresp_dist_;

    //Other parameters
    std::string output_dir_3d_results_;

    bool use_table_plane_;
    
public:
    multiviewGraph(){
        do_reverse_hyp_extension = false;
        go_3d_ = false;
        mv_keypoints_ = 0;
        opt_type_ = 0;
        gt_or_ouput_dir_ = "";
        chop_at_z_ = 1.5f;
        icp_resolution_ = 0.005f;
        icp_max_correspondence_distance_ = 0.02f;
        scene_to_scene_ = true;
        use_robot_pose_ = true;
        use_gc_s2s_ = true;

        use_unverified_single_view_hypotheses = false;

        //GO3D parameters
        go3d_color_sigma_ = 0.3f;
        go3d_outlier_regularizer_ = 3.f;
        go3d_clutter_regularizer_ = 3.f;
        go3d_clutter_radius_ = 0.04f;
        go3d_inlier_threshold_ = 0.01f;

        go3d_detect_clutter_ = true;
        go3d_add_planes_ = false;
        go3d_icp_ = true;
        go3d_icp_model_to_scene_ = false;
        go3d_use_supervoxels_ = true;

        go3d_and_icp_resolution_ = 0.005f;

        //Noise model parameters
        max_angle_ = 70.f;
        lateral_sigma_ = 0.0015f;
        nm_integration_min_weight_ = 0.25f;

        //Multiview refinement parameters
        mv_icp_ = true;
        max_keypoint_dist_mv_ = 2.5f;
        mv_iterations_ = 5;
        min_overlap_mv_ = 0.3f;
        mv_keypoints_ = 0;
        inlier_threshold_ = 0.003f;
        max_corresp_dist_ = 0.01f;

        //Other parameters
        output_dir_3d_results_ = "";

        use_table_plane_ = true;

        edge_vis.reset (new pcl::visualization::PCLVisualizer());
    }

    bool calcFeatures(Vertex &src, Graph &grph, bool use_table_plane=true);
    void estimateViewTransformationBySIFT ( const Vertex &src, const Vertex &trgt, Graph &grph, flann::Index<DistT > *flann_index, Eigen::Matrix4f &transformation, std::vector<Edge> & edges, bool use_gc=false );
    void estimateViewTransformationByRobotPose ( const Vertex &src, const Vertex &trgt, Graph &grph, Edge &edge );
    void extendHypothesis ( Graph &grph );
    std::vector<Hypothesis> extendHypothesisRecursive ( Graph &grph, Edge calling_out_edge);
    //    void calcMST ( const std::vector<Edge> &edges, const Graph &grph, std::vector<Edge> &edges_final );
    //    void createEdgesFromHypothesisMatch ( Graph &grph, std::vector<Edge> &edges );
    //    void selectLowestWeightEdgesFromParallelEdges ( const std::vector<Edge> &parallel_edges, const Graph &grph, std::vector<Edge> &single_edges );
    void createEdgesFromHypothesisMatchOnline ( const Vertex new_vertex, Graph &grph, std::vector<Edge> &edges );
    void calcEdgeWeight (Graph &grph, int max_distance=-1, float z_dist=3.f, float max_overlap=0.75f);
    void createBigPointCloud ( Graph & grph_final, pcl::PointCloud<pcl::PointXYZRGB>::Ptr & big_cloud );
    Vertex getFurthestVertex ( Graph &grph);

    std::string getSceneName()
    {
        return scene_name_;
    }
    void setSceneName(const std::string scene_name)
    {
        scene_name_ = scene_name;
    }

    void visualizeEdge (const Edge &edge, const Graph &grph);
    bool recognize (recognition_srv_definitions::multiview_recognize::Request & req, recognition_srv_definitions::multiview_recognize::Response & response);
    void loadModels();

    // getter and setter functions
    std::string models_dir() const;
    void setModels_dir(const std::string &models_dir);
    bool visualize_output() const;
    void setVisualize_output(bool visualize_output);
    bool go_3d() const;
    void setGo_3d(bool go_3d);
    int icp_iter() const;
    void setIcp_iter(int icp_iter);
    int opt_type() const;
    void setOpt_type(int opt_type);
    std::string gt_or_ouput_dir() const;
    void setGt_or_ouput_dir(const std::string &gt_or_ouput_dir);
    double chop_at_z() const;
    void setChop_at_z(double chop_at_z);
    int mv_keypoints() const;
    void setMv_keypoints(int mv_keypoints);
};

namespace multiview
{
void
nearestKSearch ( flann::Index<flann::L1<float> > * index,
                 float * descr, int descr_size,
                 int k,flann::Matrix<int> &indices,flann::Matrix<float> &distances );

template <typename Type>
void
convertToFLANN ( typename pcl::PointCloud<Type>::Ptr & cloud, flann::Matrix<float> &data );
}

#endif
