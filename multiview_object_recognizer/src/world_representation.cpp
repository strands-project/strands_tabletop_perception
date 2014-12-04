#include "world_representation.h"
#include "boost_graph_extension.h"
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
    newGraph.set_extension_mode(extension_mode_);
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

bool worldRepresentation::recognize (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr pInput,
                                     const std::string &scene_name,
                                     const std::string &view_name,
                                     const size_t &timestamp,
                                     const std::vector<double> global_trans_v,
                                     std::vector<ModelTPtr> &models_mv,
                                     std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > &transforms_mv,
                                     const std::string filepath_or_results_mv,
                                     const std::string filepath_or_results_sv)
{
    bool mv_recognize_error;

    multiviewGraph &currentMvGraph = get_current_graph(scene_name);

    Eigen::Matrix4f global_trans;
    if(global_trans_v.size() == 16 && use_robot_pose_)
    {
        for (size_t row=0; row <4; row++)
        {
            for(size_t col=0; col<4; col++)
            {
                global_trans(row, col) = global_trans_v[4*row + col];
            }
        }
        mv_recognize_error = currentMvGraph.recognize(pInput, view_name, timestamp, global_trans);//req, response);
    }
    else
    {
        std::cout << "No transform (16x1 float vector) provided. " << std::endl;
        mv_recognize_error = currentMvGraph.recognize(pInput, view_name, timestamp);
    }


    std::vector<ModelTPtr> models_sv;
    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > transforms_sv;
    std::vector<double> execution_times;
    currentMvGraph.getVerifiedHypothesesSingleView(models_sv, transforms_sv);

    currentMvGraph.getVerifiedHypotheses(models_mv, transforms_mv);
    currentMvGraph.get_times(execution_times);

    std::cout << "In the most current vertex I detected " << models_mv.size() << " verified models. " << std::endl;

    if(filepath_or_results_mv.length())
    {
        std::map<std::string, size_t> rec_models_per_id;
        for(size_t i = 0; i < models_mv.size(); i++)
        {
            std::string model_id = models_mv.at(i)->id_;
            Eigen::Matrix4f tf = transforms_mv[i];

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

            // Save multiview object recogniton result in file

            std::stringstream or_filepath_ss_mv;
            or_filepath_ss_mv << filepath_or_results_mv << "/" << view_name << "_" << model_id.substr(0, model_id.length() - 4) << "_" << num_models_per_model_id <<".txt";

            ofstream or_file;
            or_file.open (or_filepath_ss_mv.str());
            for (size_t row=0; row <4; row++)
            {
                for(size_t col=0; col<4; col++)
                {
                    or_file << tf(row, col) << " ";
                }
            }
            or_file.close();
        }

        // save measured execution times
        std::stringstream or_filepath_times;
        or_filepath_times << filepath_or_results_mv << "/" << view_name << "_times.txt";

        ofstream time_file;
        time_file.open (or_filepath_times.str());
        for (size_t time_id=0; time_id < execution_times.size(); time_id++)
        {
            time_file << execution_times[time_id] << std::endl;
        }
        time_file.close();


        // save final graph structure
        std::stringstream or_filepath_final_graph;
        or_filepath_final_graph << filepath_or_results_mv << "/" << "final_graph.dot";
        std::stringstream or_filepath_full_graph;
        or_filepath_full_graph << filepath_or_results_mv << "/" << "full_graph.dot";
        Graph currentFinalGrph, currentFullGrph;
        currentMvGraph.get_final_graph(currentFinalGrph);
        currentMvGraph.get_full_graph(currentFullGrph);
        outputgraph ( currentFinalGrph, or_filepath_final_graph.str().c_str() );
        outputgraph ( currentFullGrph, or_filepath_full_graph.str().c_str() );
    }


    if (filepath_or_results_sv.length())
    {
        std::map<std::string, size_t> rec_models_per_id_sv;
        for(size_t i = 0; i < models_sv.size(); i++)
        {
            std::string model_id = models_sv.at(i)->id_;
            Eigen::Matrix4f tf = transforms_sv[i];

            size_t num_models_per_model_id;

            std::map<std::string, size_t>::iterator it_rec_mod;
            it_rec_mod = rec_models_per_id_sv.find(model_id);
            if(it_rec_mod == rec_models_per_id_sv.end())
            {
                rec_models_per_id_sv.insert(std::pair<std::string, size_t>(model_id, 1));
                num_models_per_model_id = 0;
            }
            else
            {
                num_models_per_model_id = it_rec_mod->second;
                it_rec_mod->second++;
            }

            // Save single view object recogniton result in file
            // Save multiview object recogniton result in file

            std::stringstream or_filepath_ss_sv;
            or_filepath_ss_sv << filepath_or_results_sv << "/" << view_name << "_" << model_id.substr(0, model_id.length() - 4) << "_" << num_models_per_model_id <<".txt";

            ofstream or_file;
            or_file.open (or_filepath_ss_sv.str());
            for (size_t row=0; row <4; row++)
            {
                for(size_t col=0; col<4; col++)
                {
                    or_file << tf(row, col) << " ";
                }
            }
            or_file.close();
        }
    }
    return mv_recognize_error;
}

bool worldRepresentation::recognizeROSWrapper (recognition_srv_definitions::multiview_recognize::Request & req, recognition_srv_definitions::multiview_recognize::Response & response)
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

//    std::stringstream save_filepath;
//    save_filepath << "/media/Data/datasets/TUW/test_set/set_00017/" << view_name << ".pcd";
//    pcl::io::savePCDFileBinary(save_filepath.str(), *pInputCloud);

    std::vector<double> global_trans_v;

    for(size_t i=0; i < req.transform.size(); i++)
        global_trans_v.push_back(req.transform[i]);

    std::vector<ModelTPtr> models_mv;
    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > transforms_mv;
    bool rec_error = recognize(pInputCloud, scene_name, view_name, timestamp, global_trans_v, models_mv, transforms_mv);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pRecognizedModels (new pcl::PointCloud<pcl::PointXYZRGB>);
    for(size_t i = 0; i < models_mv.size(); i++)
    {
        std::string model_id = models_mv.at(i)->id_;
        Eigen::Matrix4f tf = transforms_mv[i];

        ConstPointInTPtr pModelCloud = models_mv.at (i)->getAssembled (0.005f);
        typename pcl::PointCloud<PointT>::Ptr pModelAligned (new pcl::PointCloud<PointT>);
        if(req.transform.size() == 16)
        {
            Eigen::Matrix4f global_trans;
            for (size_t row=0; row <4; row++)
            {
                for(size_t col=0; col<4; col++)
                {
                    global_trans(row, col) = global_trans_v[4*row + col];
                }
            }
            pcl::transformPointCloud (*pModelCloud, *pModelAligned, global_trans * transforms_mv[i]);
        }
        else
            pcl::transformPointCloud (*pModelCloud, *pModelAligned, transforms_mv[i]);
        *pRecognizedModels += *pModelAligned;

        std_msgs::String ros_string;
        ros_string.data = model_id;
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

    return rec_error;
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
