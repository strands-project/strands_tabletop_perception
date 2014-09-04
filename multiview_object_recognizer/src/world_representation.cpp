#include "world_representation.h"
#include <iostream>
#include <fstream>

multiviewGraph& worldRepresentation::get_current_graph(const std::string scene_name)
{
    for (size_t scene_id = 0; scene_id < graph_v.size(); scene_id++)
    {
        if( graph_v[scene_id].get_scene_name().compare ( scene_name) == 0 )	//--show-hypotheses-from-single-view
        {
            return graph_v[scene_id];
        }
    }

    multiviewGraph newGraph;
    newGraph.setVisualize_output(visualize_output_);
//    newGraph.setIcp_iter(icp_iter_);
    newGraph.setChop_at_z(chop_at_z_);
    newGraph.set_scene_name(scene_name);
    newGraph.setPSingleview_recognizer(pSingleview_recognizer_);
    newGraph.setSift(sift_);
    newGraph.set_scene_to_scene(scene_to_scene_);
    newGraph.set_max_vertices_in_graph(max_vertices_in_graph_);
    newGraph.set_distance_keypoints_get_discarded(distance_keypoints_get_discarded_);
    newGraph.set_visualize_output(visualize_output_);
    graph_v.push_back(newGraph);
    return graph_v.back();
}

void worldRepresentation::setPSingleview_recognizer(const boost::shared_ptr<Recognizer> &value)
{
    pSingleview_recognizer_ = value;
}


cv::Ptr<SiftGPU> worldRepresentation::sift() const
{
    return sift_;
}

void worldRepresentation::setSift(const cv::Ptr<SiftGPU> &sift)
{
    sift_ = sift;
}
bool worldRepresentation::recognize (recognition_srv_definitions::multiview_recognize::Request & req, recognition_srv_definitions::multiview_recognize::Response & response)
{
    if (req.cloud.data.size()==0)
    {
        ROS_ERROR("Point cloud is empty!");
        return false;
    }
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pInputCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(req.cloud, *pInputCloud );
    std::string scene_name = req.scene_name.data;
    std::string view_name = req.view_name.data;
    size_t timestamp = req.timestamp.data.toNSec();

    multiviewGraph &currentGraph = get_current_graph(scene_name);

    bool mv_recognize_error;

    Eigen::Matrix4f global_trans;
    if(req.transform.size() == 16)
    {
        for (size_t row=0; row <4; row++)
        {
            for(size_t col=0; col<4; col++)
            {
                global_trans(row, col) = req.transform[4*row + col];
            }
        }
        mv_recognize_error = currentGraph.recognize(pInputCloud, view_name, timestamp, global_trans);//req, response);
    }
    else
    {
        std::cout << "No transform (16x1 float vector) provided. " << std::endl;
        mv_recognize_error = currentGraph.recognize(pInputCloud, view_name, timestamp);
    }

    std::vector<ModelTPtr> models;
    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > transforms;
    currentGraph.getVerifiedHypotheses(models, transforms);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pRecognizedModels (new pcl::PointCloud<pcl::PointXYZRGB>);
    std::cout << "In the most current vertex I detected " << models.size() << " verified models. " << std::endl;

    std::map<std::string, size_t> rec_models_per_id;

    for(size_t i = 0; i < models.size(); i++)
    {
        std::string model_id = models.at(i)->id_;
        Eigen::Matrix4f tf = transforms[i];

        size_t num_models_per_model_id;

        std::map<std::string, size_t>::iterator it_rec_mod;
        it_rec_mod = rec_models_per_id.find(model_id);
        if(it_rec_mod == rec_models_per_id.end())
        {
            rec_models_per_id.insert(std::pair<std::string, size_t>(model_id, 1));
            num_models_per_model_id = 0;
        }
        else
        {
            num_models_per_model_id = it_rec_mod->second;
            it_rec_mod->second++;
        }

        // Save object recogniton result in file
        std::stringstream or_filepath_ss;
        or_filepath_ss << "/home/thomas/Projects/thomas.faeulhammer/eval/" << view_name << "_" << model_id.substr(0, model_id.length() - 4) << "_" << num_models_per_model_id <<".txt";
        ofstream or_file;
        or_file.open (or_filepath_ss.str());
        for (size_t row=0; row <4; row++)
        {
            for(size_t col=0; col<4; col++)
            {
                or_file << tf(row, col) << " ";
            }
        }
        or_file.close();

        ConstPointInTPtr pModelCloud = models.at (i)->getAssembled (0.005f);
        typename pcl::PointCloud<PointT>::Ptr pModelAligned (new pcl::PointCloud<PointT>);
        if(req.transform.size() == 16)
            pcl::transformPointCloud (*pModelCloud, *pModelAligned, global_trans * transforms[i]);
        else
            pcl::transformPointCloud (*pModelCloud, *pModelAligned, transforms[i]);
        *pRecognizedModels += *pModelAligned;

        std_msgs::String ros_string;
        ros_string.data = models.at(i)->id_;
        response.ids.push_back(ros_string);

        geometry_msgs::Transform tt;
        tt.translation.x = tf(0,3);
        tt.translation.y = tf(1,3);
        tt.translation.z = tf(2,3);

        Eigen::Matrix3f rotation = tf.block<3,3>(0,0);
        Eigen::Quaternionf q(rotation);
        tt.rotation.x = q.x();
        tt.rotation.y = q.y();
        tt.rotation.z = q.z();
        tt.rotation.w = q.w();
        response.transforms.push_back(tt);
    }

    sensor_msgs::PointCloud2  pc2;
    pcl::toROSMsg (*pRecognizedModels, pc2);
    pc2.header.frame_id = "map";
    pc2.header.stamp = req.timestamp.data;
    pc2.is_dense = false;
    vis_pc_pub_.publish(pc2);

    return mv_recognize_error;
}


// getter and setter functions
int worldRepresentation::opt_type() const
{
    return opt_type_;
}

void worldRepresentation::setOpt_type(int opt_type)
{
    opt_type_ = opt_type;
}

double worldRepresentation::chop_at_z() const
{
    return chop_at_z_;
}

void worldRepresentation::setChop_at_z(double chop_at_z)
{
    chop_at_z_ = chop_at_z;
}

std::string worldRepresentation::models_dir() const
{
    return models_dir_;
}

void worldRepresentation::setModels_dir(const std::string &models_dir)
{
    models_dir_ = models_dir;
}
