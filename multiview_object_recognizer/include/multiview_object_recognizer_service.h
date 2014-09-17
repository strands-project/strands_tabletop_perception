#ifndef MYGRAPHCLASSES_H
#define MYGRAPHCLASSES_H

#include <vector>
#include <iostream>
#include <string>
#include <sstream>
#include <opencv2/core/core.hpp>
#include <opencv2/nonfree/features2d.hpp>

#include <pcl/common/transforms.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/registration/icp.h>
#include <pcl/search/impl/flann_search.hpp>

#include <faat_pcl/3d_rec_framework/defines/faat_3d_rec_framework_defines.h>
#include <faat_pcl/3d_rec_framework/feature_wrapper/local/image/sift_local_estimator.h>
#include <faat_pcl/3d_rec_framework/pc_source/model_only_source.h>
#include <faat_pcl/registration/fast_icp_with_gc.h>
#include <faat_pcl/utils/miscellaneous.h>
#include <faat_pcl/utils/pcl_visualization_utils.h>

#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
#include <pcl_conversions.h>
#include "boost_graph_extension.h"
#include "recognition_srv_definitions/multiview_recognize.h"
#include "segmenter.h"
#include "singleview_object_recognizer.h"

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
    boost::shared_ptr<Recognizer> pSingleview_recognizer_;
    Graph grph_, grph_final_;
    std::string most_current_view_id_, scene_name_;
    boost::shared_ptr< pcl::PointCloud<PointT> > pAccumulatedKeypoints_;
    boost::shared_ptr< pcl::PointCloud<pcl::Normal> > pAccumulatedKeypointNormals_;
    std::map<std::string, faat_pcl::rec_3d_framework::ObjectHypothesis<PointT> > accumulatedHypotheses_;
    pcl::visualization::PCLVisualizer::Ptr edge_vis_;
    bool visualize_output_;
    float chop_at_z_;
    float distance_keypoints_get_discarded_;
    float icp_resolution_;
    pcl::visualization::PCLVisualizer::Ptr vis_;
    bool scene_to_scene_;
    bool use_robot_pose_;
    bool use_gc_s2s_;
    size_t max_vertices_in_graph_;
    std::vector<double> times_;

    Eigen::Matrix4f current_global_transform_;

    cv::Ptr<SiftGPU> sift_;
    
public:
    multiviewGraph(){
//        do_reverse_hyp_extension = false;
        chop_at_z_ = 1.5f;
        icp_resolution_ = 0.005f;
        scene_to_scene_ = true;
        use_robot_pose_ = false;
        use_gc_s2s_ = true;
        distance_keypoints_get_discarded_ = 0.005*0.005;
        max_vertices_in_graph_ = 4;
        visualize_output_ = false;

        pAccumulatedKeypoints_.reset (new pcl::PointCloud<PointT>);
        pAccumulatedKeypointNormals_.reset (new pcl::PointCloud<pcl::Normal>);
    }

    bool calcSiftFeatures(Vertex &src, Graph &grph);
    void estimateViewTransformationBySIFT ( const Vertex &src, const Vertex &trgt, Graph &grph, flann::Index<DistT > *flann_index, Eigen::Matrix4f &transformation, std::vector<Edge> & edges, bool use_gc=false );
    void estimateViewTransformationByRobotPose ( const Vertex &src, const Vertex &trgt, Graph &grph, Edge &edge );
    void extendHypothesis ( Graph &grph );
    void extendHypothesisRecursive ( Graph &grph, Edge calling_out_edge, std::vector<Hypothesis<PointT> > &hyp_vec);
    void extendFeatureMatchesRecursive ( Graph &grph,
                                         Vertex &vrtx_start,
                                         std::map < std::string,faat_pcl::rec_3d_framework::ObjectHypothesis<PointT> > &hypotheses,
                                         pcl::PointCloud<PointT>::Ptr keypoints,
                                         pcl::PointCloud<pcl::Normal>::Ptr keypointNormals);
    //    void calcMST ( const std::vector<Edge> &edges, const Graph &grph, std::vector<Edge> &edges_final );
    //    void createEdgesFromHypothesisMatch ( Graph &grph, std::vector<Edge> &edges );
    //    void selectLowestWeightEdgesFromParallelEdges ( const std::vector<Edge> &parallel_edges, const Graph &grph, std::vector<Edge> &single_edges );
    void createEdgesFromHypothesisMatchOnline ( const Vertex new_vertex, Graph &grph, std::vector<Edge> &edges );
    void calcEdgeWeight (Graph &grph, std::vector<Edge> &edges);
    void visualizeGraph ( const Graph & grph, pcl::visualization::PCLVisualizer::Ptr &vis);

    void set_max_vertices_in_graph(const size_t num)
    {
        max_vertices_in_graph_ = num;
    }

    void set_scene_to_scene(const bool scene_to_scene)
    {
        scene_to_scene_ = scene_to_scene;
    }

    void set_distance_keypoints_get_discarded(const double distance)
    {
        distance_keypoints_get_discarded_ = distance;
    }

    void set_visualize_output(const bool vis_output)
    {
        visualize_output_ = vis_output;
    }

    void set_scene_name(const std::string scene_name)
    {
        scene_name_ = scene_name;
    }

    std::string get_scene_name() const
    {
        return scene_name_;
    }

    void get_times(std::vector<double> &times) const
    {
        times = times_;
    }

    void visualizeEdge (const Edge &edge, const Graph &grph);

    bool recognize ( const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr inputCloud,
                     const std::string view_name,
                     const size_t timestamp);

    bool recognize ( const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr inputCloud,
                     const std::string view_name,
                     const size_t timestamp,
                     const Eigen::Matrix4f global_transform);

    // getter and setter functions
    std::string models_dir() const;
    void setModels_dir(const std::string &models_dir);
    bool visualize_output() const;
    void setVisualize_output(bool visualize_output);
    int opt_type() const;
    void setOpt_type(int opt_type);
    double chop_at_z() const;
    void setChop_at_z(double chop_at_z);
    int mv_keypoints() const;
    void setMv_keypoints(int mv_keypoints);
    void setPSingleview_recognizer(const boost::shared_ptr<Recognizer> &value);
    cv::Ptr<SiftGPU> sift() const;
    void setSift(const cv::Ptr<SiftGPU> &sift);

    bool getVerifiedHypotheses(std::vector<ModelTPtr> &models, std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > &transforms)
    {
        models.clear();
        transforms.clear();

        if(num_vertices(grph_))
        {
            std::pair<vertex_iter, vertex_iter> vp;
            for ( vp = vertices ( grph_ ); vp.first != vp.second; ++vp.first )
            {
                if (grph_[*vp.first].pScenePCl->header.frame_id.compare(most_current_view_id_) == 0)
                {
                    for(size_t i=0; i < grph_[*vp.first].hypothesis_mv_.size(); i++)
                    {
                        if(grph_[*vp.first].hypothesis_mv_[i].verified_)
                        {
                            models.push_back(grph_[*vp.first].hypothesis_mv_[i].model_);
                            transforms.push_back(grph_[*vp.first].hypothesis_mv_[i].transform_);
                        }
                    }
                    return true;
                }
            }
        }
        PCL_ERROR("There is no most current vertex in the graph.");
        return false;
    }


    bool getVerifiedHypothesesSingleView(std::vector<ModelTPtr> &models, std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > &transforms)
    {
        models.clear();
        transforms.clear();

        if(num_vertices(grph_))
        {
            std::pair<vertex_iter, vertex_iter> vp;
            for ( vp = vertices ( grph_ ); vp.first != vp.second; ++vp.first )
            {
                if (grph_[*vp.first].pScenePCl->header.frame_id.compare(most_current_view_id_) == 0)
                {
                    for(size_t i=0; i < grph_[*vp.first].hypothesis_sv_.size(); i++)
                    {
                        if(grph_[*vp.first].hypothesis_sv_[i].verified_)
                        {
                            models.push_back(grph_[*vp.first].hypothesis_sv_[i].model_);
                            transforms.push_back(grph_[*vp.first].hypothesis_sv_[i].transform_);
                        }
                    }
                    return true;
                }
            }
        }
        PCL_ERROR("There is no most current vertex in the graph.");
        return false;
    }

};

namespace multiview
{
void
nearestKSearch ( flann::Index<flann::L1<float> > * index,
                 float * descr, int descr_size,
                 int k,flann::Matrix<int> &indices,flann::Matrix<float> &distances );

template <typename Type>
void
convertToFLANN ( const typename pcl::PointCloud<Type>::ConstPtr & cloud, flann::Matrix<float> &data );
}

#endif
