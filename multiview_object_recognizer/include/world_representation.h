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
    bool visualize_output_;
    bool go_3d_;
    int icp_iter_;
    int opt_type_;
    std::string gt_or_ouput_dir_;
    double chop_at_z_;
    int mv_keypoints_;
    std::string models_dir_;

public:
    bool recognize (recognition_srv_definitions::multiview_recognize::Request & req, recognition_srv_definitions::multiview_recognize::Response & response);
    multiviewGraph& get_current_graph(const std::string scene_name);

    // getter and setter functions
    int icp_iter() const;
    void setIcp_iter(int icp_iter);
    bool visualize_output() const;
    void setVisualize_output(bool visualize_output);
    bool go_3d() const;
    void setGo_3d(bool go_3d);
    int opt_type() const;
    void setOpt_type(int opt_type);
    std::string gt_or_ouput_dir() const;
    void setGt_or_ouput_dir(const std::string &gt_or_ouput_dir);
    double chop_at_z() const;
    void setChop_at_z(double chop_at_z);
    int mv_keypoints() const;
    void setMv_keypoints(int mv_keypoints);
    std::string models_dir() const;
    void setModels_dir(const std::string &models_dir);
    void setPSingleview_recognizer(const boost::shared_ptr<Recognizer> &value);
};

#endif
