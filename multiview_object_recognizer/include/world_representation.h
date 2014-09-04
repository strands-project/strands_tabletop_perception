#ifndef WORLD_REPRESENTATION_H
#define WORLD_REPRESENTATION_H

#define EIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET
#include "multiview_object_recognizer_service.h"
#include "singleview_object_recognizer.h"

class worldRepresentation
{
private:
    boost::shared_ptr<Recognizer> pSingleview_recognizer_;
    std::vector <multiviewGraph > graph_v;
    bool visualize_output_, scene_to_scene_;
    int icp_iter_;
    int opt_type_;
    double chop_at_z_;
    std::string models_dir_;
    cv::Ptr<SiftGPU> sift_;
    ros::Publisher vis_pc_pub_;
    size_t max_vertices_in_graph_;
    double distance_keypoints_get_discarded_;
    pcl::visualization::PCLVisualizer::Ptr vis;

public:
    worldRepresentation()
    {
        distance_keypoints_get_discarded_ = 0.005*0.005;
        max_vertices_in_graph_ = 4;
        visualize_output_ = false;
        scene_to_scene_ = true;
        vis.reset ( new pcl::visualization::PCLVisualizer ( "vis" ) );
    }

    bool recognize (recognition_srv_definitions::multiview_recognize::Request & req, recognition_srv_definitions::multiview_recognize::Response & response);
    multiviewGraph& get_current_graph(const std::string scene_name);

    // getter and setter functions
    void set_vis_pc_pub(const ros::Publisher &vis_pc_pub)
    {
        vis_pc_pub_ = vis_pc_pub;
    }

    int icp_iter() const;
    void setIcp_iter(int icp_iter);
    bool visualize_output() const;
    void setVisualize_output(bool visualize_output);
    int opt_type() const;
    void setOpt_type(int opt_type);
    double chop_at_z() const;
    void setChop_at_z(double chop_at_z);
    std::string models_dir() const;
    void setModels_dir(const std::string &models_dir);
    void setPSingleview_recognizer(const boost::shared_ptr<Recognizer> &value);
    cv::Ptr<SiftGPU> sift() const;
    void setSift(const cv::Ptr<SiftGPU> &sift);
    void set_scene_to_scene(const bool scene_to_scene)
    {
        scene_to_scene_ = scene_to_scene;
    }
    void set_max_vertices_in_graph(const size_t num)
    {
        max_vertices_in_graph_ = num;
    }

    void set_distance_keypoints_get_discarded(const double distance)
    {
        distance_keypoints_get_discarded_ = distance;
    }

    void set_visualize_output(const bool vis_output)
    {
        visualize_output_ = vis_output;
    }
};

#endif
