#include "world_representation_ros.h"

#include "ros/ros.h"
#include <iostream>
#include <sstream>
#include <string>
#include <v4r/utils/filesystem_utils.h>
#include <v4r/ORRecognition/include/singleview_object_recognizer.h>
#include <v4r/ORRecognition/include/world_representation.h>

#include <pcl/io/pcd_io.h>

//#define USE_WILLOW_DATASET_FOR_EVAL

ros::ServiceClient client_;

int main (int argc, char **argv)
{
    boost::shared_ptr<ros::NodeHandle> nh;
    std::string models_dir;
    bool visualize_output;
    bool scene_to_scene;
    bool use_robot_pose;
    bool do_eval;
    int icp_iter;
    int icp_type;
    int opt_type;
    int extension_mode;
    bool play_sequence_randomly = false;
    worldRepresentationROS myWorld;
    boost::shared_ptr<Recognizer> pSingleview_recognizer (new Recognizer());

    double chop_at_z;
    double distance_keypoints_get_discarded;
    std::string training_dir_sift, training_dir_shot, sift_structure, training_dir_ourcvfh;
    bool do_sift=false, do_ourcvfh=false, do_shot=false;
    int max_vertices_in_graph;

    ros::init ( argc, argv, "multiview_object_recognizer_node" );
    nh.reset( new ros::NodeHandle ( "~" ) );

    if ( ! nh->getParam ( "models_dir", models_dir ))
    {
        std::cout << "No models_dir specified. " << std::endl;
    }

    nh->getParam ( "training_dir_sift", training_dir_sift);
    nh->getParam ( "training_dir_shot", training_dir_shot);
    nh->getParam ( "recognizer_structure_sift", sift_structure);
    nh->getParam ( "training_dir_ourcvfh", training_dir_ourcvfh);

    if (models_dir.compare ("") == 0)
    {
        PCL_ERROR ("Set -models_dir option in the command line, ABORTING");
        return -1;
    }

    //-----Init-SIFT-GPU-Context--------
    static char kw[][16] = {"-m", "-fo", "-1", "-s", "-v", "1", "-pack"};
    char * argvv[] = {kw[0], kw[1], kw[2], kw[3],kw[4],kw[5],kw[6], NULL};

    int argcc = sizeof(argvv) / sizeof(char*);
    cv::Ptr<SiftGPU> sift = new SiftGPU ();
    sift->ParseParam (argcc, argvv);

    //create an OpenGL context for computation
    if (sift->CreateContextGL () != SiftGPU::SIFTGPU_FULL_SUPPORTED)
        throw std::runtime_error ("PSiftGPU::PSiftGPU: No GL support!");

    pSingleview_recognizer->setTraining_dir_ourcvfh(training_dir_ourcvfh);
    pSingleview_recognizer->setTraining_dir_sift(training_dir_sift);
    pSingleview_recognizer->setTraining_dir_shot(training_dir_shot);
    pSingleview_recognizer->setModels_dir(models_dir);
    pSingleview_recognizer->setSift_structure(sift_structure);

    if(!(nh->getParam ( "do_eval", do_eval)))
        do_eval = false;

    if(nh->getParam ( "icp_iterations", icp_iter))
        pSingleview_recognizer->set_icp_iterations(icp_iter);

    if(nh->getParam ( "icp_type", icp_type))
        pSingleview_recognizer->set_icp_type(icp_type);

    if(nh->getParam ( "do_sift", do_sift))
        pSingleview_recognizer->set_do_sift(do_sift);

    if(nh->getParam ( "do_shot", do_shot))
        pSingleview_recognizer->set_do_shot(do_shot);

    if(do_sift)
        pSingleview_recognizer->set_sift(sift);

    if(nh->getParam ( "do_ourcvfh", do_ourcvfh))
        pSingleview_recognizer->set_do_ourcvfh(do_ourcvfh);

    if (do_sift && training_dir_sift.compare ("") == 0)
    {
        PCL_ERROR ("do_sift is activated but training_dir_sift_ is empty! Set -training_dir_sift option in the command line, ABORTING");
        return -1;
    }

    if (do_ourcvfh && training_dir_ourcvfh.compare ("") == 0)
    {
        PCL_ERROR ("do_ourcvfh is activated but training_dir_ourcvfh_ is empty! Set -training_dir_ourcvfh option in the command line, ABORTING");
        return -1;
    }

    if (do_shot && training_dir_shot.compare ("") == 0)
    {
        PCL_ERROR ("do_shot is activated but training_dir_ourcvfh_ is empty! Set -training_dir_shot option in the command line, ABORTING");
        return -1;
    }

    nh->getParam ( "cg_size_thresh", pSingleview_recognizer->cg_params_.cg_size_threshold_);
    nh->getParam ( "cg_size", pSingleview_recognizer->cg_params_.cg_size_);
    nh->getParam ( "cg_ransac_threshold", pSingleview_recognizer->cg_params_.ransac_threshold_);
    nh->getParam ( "cg_dist_for_clutter_factor", pSingleview_recognizer->cg_params_.dist_for_clutter_factor_);
    nh->getParam ( "cg_max_taken", pSingleview_recognizer->cg_params_.max_taken_);
    nh->getParam ( "cg_max_time_for_cliques_computation", pSingleview_recognizer->cg_params_.max_time_for_cliques_computation_);
    nh->getParam ( "cg_dot_distance", pSingleview_recognizer->cg_params_.dot_distance_);

    nh->getParam ( "hv_resolution", pSingleview_recognizer->hv_params_.resolution_);
    nh->getParam ( "hv_inlier_threshold", pSingleview_recognizer->hv_params_.inlier_threshold_);
    nh->getParam ( "hv_radius_clutter", pSingleview_recognizer->hv_params_.radius_clutter_);
    nh->getParam ( "hv_regularizer", pSingleview_recognizer->hv_params_.regularizer_);
    nh->getParam ( "hv_clutter_regularizer", pSingleview_recognizer->hv_params_.clutter_regularizer_);
    nh->getParam ( "hv_occlusion_threshold", pSingleview_recognizer->hv_params_.occlusion_threshold_);
    nh->getParam ( "hv_optimizer_type", pSingleview_recognizer->hv_params_.optimizer_type_);
    nh->getParam ( "hv_color_sigma_l", pSingleview_recognizer->hv_params_.color_sigma_l_);
    nh->getParam ( "hv_color_sigma_ab", pSingleview_recognizer->hv_params_.color_sigma_ab_);
    nh->getParam ( "hv_use_supervoxels", pSingleview_recognizer->hv_params_.use_supervoxels_);
    nh->getParam ( "hv_detect_clutter", pSingleview_recognizer->hv_params_.detect_clutter_);
    nh->getParam ( "hv_ignore_color", pSingleview_recognizer->hv_params_.ignore_color_);


    if(nh->getParam ( "opt_type", opt_type))
        myWorld.setOpt_type(opt_type);

    if(nh->getParam ( "chop_z", chop_at_z))
    {
        myWorld.setChop_at_z(chop_at_z);
        pSingleview_recognizer->setChop_at_z(chop_at_z);
    }

    if(nh->getParam ( "scene_to_scene", scene_to_scene))
        myWorld.set_scene_to_scene(scene_to_scene);

    if(nh->getParam ( "use_robot_pose", use_robot_pose))
        myWorld.set_use_robot_pose(use_robot_pose);

    if(nh->getParam ( "visualize_output", visualize_output))
        myWorld.set_visualize_output(visualize_output);

    if(nh->getParam ( "extension_mode", extension_mode))
        myWorld.set_extension_mode(extension_mode);

    if(nh->getParam ( "max_vertices_in_graph", max_vertices_in_graph))
        myWorld.set_max_vertices_in_graph(max_vertices_in_graph);

    if(nh->getParam ( "distance_keypoints_get_discarded", distance_keypoints_get_discarded))
        myWorld.set_distance_keypoints_get_discarded(distance_keypoints_get_discarded);

    pSingleview_recognizer->initialize();

    myWorld.setModels_dir(models_dir);
    myWorld.setPSingleview_recognizer(pSingleview_recognizer);
    myWorld.setSift(sift);

    std::cout << "=====Started recognizer with following parameters:====="
              << "cg_size_thresh: " << pSingleview_recognizer->cg_params_.cg_size_threshold_ << std::endl
              << "cg_size: " << pSingleview_recognizer->cg_params_.cg_size_ << std::endl
              << "cg_ransac_threshold: " << pSingleview_recognizer->cg_params_.ransac_threshold_ << std::endl
              << "cg_dist_for_clutter_factor: " << pSingleview_recognizer->cg_params_.dist_for_clutter_factor_ << std::endl
              << "cg_max_taken: " << pSingleview_recognizer->cg_params_.max_taken_ << std::endl
              << "cg_max_time_for_cliques_computation: " << pSingleview_recognizer->cg_params_.max_time_for_cliques_computation_ << std::endl
              << "cg_dot_distance: " << pSingleview_recognizer->cg_params_.dot_distance_ << std::endl
              << "hv_resolution: " << pSingleview_recognizer->hv_params_.resolution_ << std::endl
              << "hv_inlier_threshold: " << pSingleview_recognizer->hv_params_.inlier_threshold_ << std::endl
              << "hv_radius_clutter: " << pSingleview_recognizer->hv_params_.radius_clutter_ << std::endl
              << "hv_regularizer: " << pSingleview_recognizer->hv_params_.regularizer_ << std::endl
              << "hv_clutter_regularizer: " << pSingleview_recognizer->hv_params_.clutter_regularizer_ << std::endl
              << "hv_occlusion_threshold: " << pSingleview_recognizer->hv_params_.occlusion_threshold_ << std::endl
              << "hv_optimizer_type: " << pSingleview_recognizer->hv_params_.optimizer_type_ << std::endl
              << "hv_color_sigma_l: " << pSingleview_recognizer->hv_params_.color_sigma_l_ << std::endl
              << "hv_color_sigma_ab: " << pSingleview_recognizer->hv_params_.color_sigma_ab_ << std::endl
              << "hv_use_supervoxels: " << pSingleview_recognizer->hv_params_.use_supervoxels_ << std::endl
              << "hv_detect_clutter: " << pSingleview_recognizer->hv_params_.detect_clutter_ << std::endl
              << "hv_ignore_color: " << pSingleview_recognizer->hv_params_.ignore_color_ << std::endl
              << "opt_type: " << opt_type << std::endl
              << "chop_z: " << chop_at_z << std::endl
              << "scene_to_scene: " << scene_to_scene << std::endl
              << "max_vertices_in_graph: " << max_vertices_in_graph << std::endl
              << "distance_keypoints_get_discarded: " << distance_keypoints_get_discarded << std::endl
              << "icp_iterations: " << icp_iter << std::endl
              << "icp_type: " << icp_type << std::endl
              << "icp_voxel_size: " << pSingleview_recognizer->hv_params_.resolution_ << std::endl
              << "do_sift: " << do_sift << std::endl
              << "do_shot: " << do_shot << std::endl
              << "do_ourcvfh: " << do_ourcvfh << std::endl
              << "do_eval: " << do_eval << std::endl
              << "extension_mode: " << extension_mode << std::endl
              << "====================" << std::endl << std::endl;


    if(!do_eval)
    {
        //client_ = n_->serviceClient<recognition_srv_definitions::recognize> ( "/recognition_service/mp_recognition" );
        ros::ServiceServer ros_mv_rec_server;
        ros_mv_rec_server = nh->advertiseService("multiview_recognotion_service", &worldRepresentationROS::recognizeROSWrapper, &myWorld);

        ros::Publisher vis_pc_pub;
        vis_pc_pub = nh->advertise<sensor_msgs::PointCloud2>( "multiview_recognized_objects", 0 );
        myWorld.set_vis_pc_pub(vis_pc_pub);

        ROS_INFO("Multiview object recognizer is ready to get service callsssss.");
        ros::spin();
    }
    else    // do some offline evaluation (with files saved locally)
    {
        int id = 0;

//        for (pSingleview_recognizer->hv_params_.regularizer_ = 1; pSingleview_recognizer->hv_params_.regularizer_ <= 7; pSingleview_recognizer->hv_params_.regularizer_+=2)
        {
//        for (pSingleview_recognizer->hv_params_.color_sigma_l_ = 0.2; pSingleview_recognizer->hv_params_.color_sigma_l_ <= 0.4; pSingleview_recognizer->hv_params_.color_sigma_l_ +=0.2)
        {
//            for (pSingleview_recognizer->hv_params_.clutter_regularizer_ = 1;
//                 pSingleview_recognizer->hv_params_.clutter_regularizer_ <= 5;
//                 pSingleview_recognizer->hv_params_.clutter_regularizer_ += 2)
            {

//                if ((pSingleview_recognizer->hv_params_.regularizer_ == 5 &&
//                     pSingleview_recognizer->hv_params_.color_sigma_l_ == 0.2 &&
//                     pSingleview_recognizer->hv_params_.clutter_regularizer_ <= 1.5))
////                    || (pSingleview_recognizer->cg_params_.cg_size_ ==0.01 && pSingleview_recognizer->cg_params_.cg_size_threshold_ == 4
////                       && pSingleview_recognizer->hv_params_.clutter_regularizer_ < 1.5))
//                    continue;


        std::string dataset_path, sequence_name_provided;
        const std::string transform_prefix_ = "transformation_";

        if(!nh->getParam("dataset_path", dataset_path))
            ROS_ERROR("No dataset path given (arg \"dataset_path\"). ");

        if(!nh->getParam("sequence_name", sequence_name_provided))
            ROS_ERROR("No sequence name given (arg \"sequence_name\"). ");

        std::stringstream eval_path_ss;
        boost::filesystem::path eval_folderpath;
        do
        {
            eval_path_ss.str("");
            eval_path_ss << "/home/thomas/Projects/thomas.faeulhammer/" << "eval_" << id << "/";
            eval_folderpath = eval_path_ss.str();
            id++;
        }while(boost::filesystem::exists(eval_folderpath) );
        boost::filesystem::create_directory(eval_folderpath);

        std::string eval_path = eval_path_ss.str();

        std::vector < std::string > scene_folder;
        std::string start = "";
        v4r::utils::getFoldersInDirectory(dataset_path, start, scene_folder);

        std::cout << "There are " << scene_folder.size() << " folders in directory " << dataset_path << "." << std::endl;

        for(size_t seq_id=0; seq_id < scene_folder.size(); seq_id++)
        {
            std::stringstream seq_path_ss;
            //seq_name_ss << "set_" << setw(5) << setfill('0') << seq_id;
            //seq_name_ss << "T_" << setw(2) << setfill('0') << seq_id << "_willow_dataset";
            seq_path_ss << dataset_path << "/" << scene_folder[seq_id];

            if(sequence_name_provided.length() && sequence_name_provided.compare(scene_folder[seq_id])!=0)
                continue;

//            std::stringstream scenes_dir_ss;
//            scenes_dir_ss << dataset_path << "/" << seq_name_ss.str();
//            std::string scenes_dir = scenes_dir_ss.str();


            std::cout << "Starting eval for " << seq_path_ss.str() << std::endl;
            std::vector < std::string > files_intern;
            v4r::utils::getFilesInDirectory (seq_path_ss.str(), files_intern, "", ".*.pcd", true);

            if(play_sequence_randomly)
                std::random_shuffle(files_intern.begin(), files_intern.end());
            else
                std::sort(files_intern.begin(), files_intern.end());

            if (files_intern.size())
            {
                size_t sub_id_mv = 0;
                std::stringstream or_folderpath_mv_ss;
                boost::filesystem::path or_folderpath_mv;
                do
                {
                    or_folderpath_mv_ss.str(std::string());
                    or_folderpath_mv_ss << eval_path << scene_folder[seq_id]  << "_" << sub_id_mv << "_mv/";
                    or_folderpath_mv = or_folderpath_mv_ss.str();
                    sub_id_mv++;
                }while(boost::filesystem::exists(or_folderpath_mv) );
                boost::filesystem::create_directory(or_folderpath_mv);


                size_t sub_id_sv = 0;
                std::stringstream or_folderpath_sv_ss;
                boost::filesystem::path or_folderpath_sv;
                do
                {
                    or_folderpath_sv_ss.str(std::string());
                    or_folderpath_sv_ss << eval_path << scene_folder[seq_id]  << "_" << sub_id_sv << "_sv/";
                    or_folderpath_sv = or_folderpath_sv_ss.str();
                    sub_id_sv++;
                }while(boost::filesystem::exists(or_folderpath_sv) );
                boost::filesystem::create_directory(or_folderpath_sv);


                std::stringstream param_file_text;
                param_file_text
                        << "cg_size_thresh: " << pSingleview_recognizer->cg_params_.cg_size_threshold_ << std::endl
                        << "cg_size: " << pSingleview_recognizer->cg_params_.cg_size_ << std::endl
                        << "cg_ransac_threshold: " << pSingleview_recognizer->cg_params_.ransac_threshold_ << std::endl
                        << "cg_dist_for_clutter_factor: " << pSingleview_recognizer->cg_params_.dist_for_clutter_factor_ << std::endl
                        << "cg_max_taken: " << pSingleview_recognizer->cg_params_.max_taken_ << std::endl
                        << "cg_max_time_for_cliques_computation: " << pSingleview_recognizer->cg_params_.max_time_for_cliques_computation_ << std::endl
                        << "cg_dot_distance: " << pSingleview_recognizer->cg_params_.dot_distance_ << std::endl
                        << "hv_resolution: " << pSingleview_recognizer->hv_params_.resolution_ << std::endl
                        << "hv_inlier_threshold: " << pSingleview_recognizer->hv_params_.inlier_threshold_ << std::endl
                        << "hv_radius_clutter: " << pSingleview_recognizer->hv_params_.radius_clutter_ << std::endl
                        << "hv_regularizer: " << pSingleview_recognizer->hv_params_.regularizer_ << std::endl
                        << "hv_clutter_regularizer: " << pSingleview_recognizer->hv_params_.clutter_regularizer_ << std::endl
                        << "hv_occlusion_threshold: " << pSingleview_recognizer->hv_params_.occlusion_threshold_ << std::endl
                        << "hv_optimizer_type: " << pSingleview_recognizer->hv_params_.optimizer_type_ << std::endl
                        << "hv_color_sigma_l: " << pSingleview_recognizer->hv_params_.color_sigma_l_ << std::endl
                        << "hv_color_sigma_ab: " << pSingleview_recognizer->hv_params_.color_sigma_ab_ << std::endl
                        << "hv_use_supervoxels: " << pSingleview_recognizer->hv_params_.use_supervoxels_ << std::endl
                        << "hv_detect_clutter: " << pSingleview_recognizer->hv_params_.detect_clutter_ << std::endl
                        << "hv_ignore_color: " << pSingleview_recognizer->hv_params_.ignore_color_ << std::endl
                        << "opt_type: " << opt_type << std::endl
                        << "chop_z: " << chop_at_z << std::endl
                        << "scene_to_scene: " << scene_to_scene << std::endl
                        << "max_vertices_in_graph: " << max_vertices_in_graph << std::endl
                        << "distance_keypoints_get_discarded: " << distance_keypoints_get_discarded << std::endl
                        << "icp_iterations: " << icp_iter << std::endl
                        << "icp_type: " << icp_type << std::endl
                        << "icp_voxel_size: " << pSingleview_recognizer->hv_params_.resolution_ << std::endl
                        << "do_sift: " << do_sift << std::endl
                        << "do_shot: " << do_shot << std::endl
                        << "do_ourcvfh: " << do_ourcvfh << std::endl
                        << "do_eval: " << do_eval << std::endl
                        << "extension_mode: " << extension_mode << std::endl;

                std::stringstream or_filepath_parameter_ss_sv;
                or_filepath_parameter_ss_sv << or_folderpath_sv_ss.str() << "parameter.nfo";
                ofstream or_file;
                or_file.open (or_filepath_parameter_ss_sv.str().c_str());
                or_file << param_file_text.str();
                or_file.close();

                std::stringstream or_filepath_parameter_ss_mv;
                or_filepath_parameter_ss_mv << or_folderpath_mv_ss.str() << "parameter.nfo";
                or_file.open (or_filepath_parameter_ss_mv.str().c_str());
                or_file << param_file_text.str();
                or_file.close();

                or_filepath_parameter_ss_mv.str(std::string());
                or_filepath_parameter_ss_mv << or_folderpath_mv_ss.str() << "view_temporal_order.nfo";
                or_file.open (or_filepath_parameter_ss_mv.str().c_str());
                for (size_t file_id=0; file_id < files_intern.size(); file_id++)
                {
                    or_file << files_intern[file_id] << std::endl;
                }
                or_file.close();

                for (size_t file_id=0; file_id < files_intern.size(); file_id++)
                {
                    std::stringstream full_file_name;
                    full_file_name << dataset_path << "/" << scene_folder[seq_id] << "/" << files_intern[file_id];
                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pScene (new pcl::PointCloud<pcl::PointXYZRGB>);
                    pcl::io::loadPCDFile(full_file_name.str(), *pScene);

                    std::vector<double> transform;
                    transform.clear();

                    std::stringstream transform_ss;
#ifndef USE_WILLOW_DATASET_FOR_EVAL
                    transform_ss << dataset_path << "/" << scene_folder[seq_id] << "/" << transform_prefix_ << files_intern[file_id].substr(0, files_intern[file_id].length() - 3) << "txt";
#else
                    transform_ss << dataset_path << "/" << scene_folder[seq_id] << "/" << "pose_" << files_intern[file_id].substr(6, files_intern[file_id].length() - 3 - 6 ) << "txt";
#endif
                    std::cout << "Checking if path " << transform_ss.str() << " for transform exists. " << std::endl;

                    if ( boost::filesystem::exists( transform_ss.str() ) )
                    {
                        std::cout << "File exists." << std::endl;
                        std::ifstream is(transform_ss.str().c_str());

#ifdef USE_WILLOW_DATASET_FOR_EVAL
                        std::string s;
                        std::vector<std::string> file_parts;
                        std::getline( is, s );
                        std::istringstream ss( s );
                        std::vector<double> numbers;
                        while (ss)
                        {
                          std::string s;
                          if (!std::getline( ss, s, ' ' )) break;
                          file_parts.push_back( s );
                          if(file_parts.size()>1)
                              numbers.push_back(atof(s.c_str()));
                        }
#else
                        std::istream_iterator<double> start(is), end;
                        std::vector<double> numbers(start, end);
                        std::cout << "Read " << numbers.size() << " numbers" << std::endl;
#endif
                        // print the numbers to stdout
                        std::cout << "Transform to world coordinate system: " << std::endl;
                        for(size_t i=0; i<numbers.size(); i++)
                        {
                            std::cout << numbers[i] << " ";
                            transform.push_back(numbers[i]);
                        }
                        std::cout << std::endl;
                    }
                    else
                    {
                        std::cout << "File does not exist. Using it without world transform." << std::endl;
                    }

                    std::vector<ModelTPtr> models_mv;
                    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > transforms_mv;
                    myWorld.recognize(pScene,
                                      scene_folder[seq_id],
                                      files_intern[file_id].substr(0, files_intern[file_id].length() - 3 - 1),
                                      ros::Time::now().nsec,
                                      transform,
                                      models_mv,
                                      transforms_mv,
                                      or_folderpath_mv_ss.str(),
                                      or_folderpath_sv_ss.str());

                    if ( models_mv.size() == 0 )
                    {
                        std::cout << "I didn't detect any object in the current scene." << std::endl;
                    }
                    else
                    {
                        for ( size_t i=0; i < models_mv.size(); i++ )
                        {
                            std::cout << "I detected object " << models_mv.at(i)->id_ << " in the scene." << std::endl;
                        }
                    }
                }
            }
        myWorld.clear();    // to reduce memory load
    }
    }
    }
}
    }

    return 0;
}
