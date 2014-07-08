#include "world_representation.h"

multiviewGraph& worldRepresentation::get_current_graph(const std::string scene_name)
{
    for (size_t scene_id = 0; scene_id < graph_v.size(); scene_id++)
    {
        if( graph_v[scene_id].getSceneName().compare ( scene_name) == 0 )	//--show-hypotheses-from-single-view
        {
            return graph_v[scene_id];
        }
    }

    multiviewGraph newGraph;
    newGraph.setModels_dir(models_dir_);
    newGraph.setVisualize_output(visualize_output_);
    newGraph.setGo_3d(go_3d_);
    newGraph.setGt_or_ouput_dir(gt_or_ouput_dir_);
    newGraph.setIcp_iter(icp_iter_);
    newGraph.setMv_keypoints(mv_keypoints_);
    newGraph.setOpt_type(opt_type_);
    newGraph.setChop_at_z(chop_at_z_);
    newGraph.setSceneName(scene_name);
    newGraph.loadModels();
    graph_v.push_back(newGraph);
    return graph_v.back();
}

bool worldRepresentation::recognize (recognition_srv_definitions::multiview_recognize::Request & req, recognition_srv_definitions::multiview_recognize::Response & response)
{

    if (req.cloud.data.size()==0)
    {
        ROS_ERROR("Point cloud is empty!");
        return false;
    }

    std::string scene_name = req.scene_name.data;

    multiviewGraph &currentGraph = get_current_graph(scene_name);
    return currentGraph.recognize(req, response);
}


// getter and setter functions
int worldRepresentation::icp_iter() const
{
    return icp_iter_;
}

void worldRepresentation::setIcp_iter(int icp_iter)
{
    icp_iter_ = icp_iter;
}

bool worldRepresentation::visualize_output() const
{
    return visualize_output_;
}

void worldRepresentation::setVisualize_output(bool visualize_output)
{
    visualize_output_ = visualize_output;
}

bool worldRepresentation::go_3d() const
{
    return go_3d_;
}

void worldRepresentation::setGo_3d(bool go_3d)
{
    go_3d_ = go_3d;
}

int worldRepresentation::opt_type() const
{
    return opt_type_;
}

void worldRepresentation::setOpt_type(int opt_type)
{
    opt_type_ = opt_type;
}

std::string worldRepresentation::gt_or_ouput_dir() const
{
    return gt_or_ouput_dir_;
}

void worldRepresentation::setGt_or_ouput_dir(const std::string &gt_or_ouput_dir)
{
    gt_or_ouput_dir_ = gt_or_ouput_dir;
}

double worldRepresentation::chop_at_z() const
{
    return chop_at_z_;
}

void worldRepresentation::setChop_at_z(double chop_at_z)
{
    chop_at_z_ = chop_at_z;
}

int worldRepresentation::mv_keypoints() const
{
    return mv_keypoints_;
}

void worldRepresentation::setMv_keypoints(int mv_keypoints)
{
    mv_keypoints_ = mv_keypoints;
}


std::string worldRepresentation::models_dir() const
{
    return models_dir_;
}

void worldRepresentation::setModels_dir(const std::string &models_dir)
{
    models_dir_ = models_dir;
}
