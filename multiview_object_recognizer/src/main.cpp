#include "singleview_object_recognizer.h"
#include "world_representation.h"

#include "ros/ros.h"
#include <iostream>

#include <pcl/io/pcd_io.h>
ros::ServiceClient client_;
boost::shared_ptr<ros::NodeHandle> n_;

int main (int argc, char **argv)
{
    std::string models_dir;
    bool visualize_output;
    bool scene_to_scene;
    int icp_iter;
    int icp_type;
    double icp_voxel_size;
    int opt_type;

    // HV Params
    double resolution;
    double inlier_threshold;
    double radius_clutter;
    double regularizer;
    double clutter_regularizer;
    double occlusion_threshold;
    int optimizer_type;
    double color_sigma_l;
    double color_sigma_ab;


    // CG PARAMS
    int cg_size_threshold;
    double cg_size;
    double ransac_threshold;
    double dist_for_clutter_factor;
    int max_taken;
    double max_time_for_cliques_computation;
    double dot_distance;

    double chop_at_z;
    double distance_keypoints_get_discarded;
    std::string training_dir_sift, training_dir_shot, sift_structure, training_dir_ourcvfh;
    bool do_sift=false, do_ourcvfh=false, do_shot=false, ignore_color;
    int max_vertices_in_graph;

    ros::init ( argc, argv, "multiview_object_recognizer_node" );
    n_.reset( new ros::NodeHandle ( "~" ) );

    if ( ! n_->getParam ( "models_dir", models_dir ))
    {
        std::cout << "No models_dir specified. " << std::endl;
    }

    n_->getParam ( "training_dir_sift", training_dir_sift);
    n_->getParam ( "training_dir_shot", training_dir_shot);
    n_->getParam ( "recognizer_structure_sift", sift_structure);
    n_->getParam ( "training_dir_ourcvfh", training_dir_ourcvfh);

    std::cout << chop_at_z << ", " << ignore_color << std::endl;
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


    boost::shared_ptr<Recognizer> pSingleview_recognizer (new Recognizer());
    pSingleview_recognizer->setTraining_dir_ourcvfh(training_dir_ourcvfh);
    pSingleview_recognizer->setTraining_dir_sift(training_dir_sift);
    pSingleview_recognizer->setTraining_dir_shot(training_dir_shot);
    pSingleview_recognizer->setModels_dir(models_dir);
    pSingleview_recognizer->setSift_structure(sift_structure);


    if(n_->getParam ( "icp_iterations", icp_iter))
        pSingleview_recognizer->set_icp_iterations(icp_iter);

    if(n_->getParam ( "icp_type", icp_type))
        pSingleview_recognizer->set_icp_type(icp_type);

    if(n_->getParam ( "icp_voxel_size", icp_voxel_size))
        pSingleview_recognizer->set_icp_type(icp_voxel_size);

    if(n_->getParam ( "do_sift", do_sift))
        pSingleview_recognizer->set_do_sift(do_sift);


    if(n_->getParam ( "do_shot", do_shot))
        pSingleview_recognizer->set_do_shot(do_shot);

    if(do_sift)
        pSingleview_recognizer->set_sift(sift);

    if(n_->getParam ( "do_ourcvfh", do_ourcvfh))
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

    if(n_->getParam ( "cg_size_thresh", cg_size_threshold)) // For correspondence grouping (default: 5, minimum: 3), the higher, the fewer hypotheses are constructed
        pSingleview_recognizer->set_cg_size_threshold(cg_size_threshold);

    if(n_->getParam ( "cg_size", cg_size))
        pSingleview_recognizer->set_cg_size(cg_size);

    if(n_->getParam ( "cg_ransac_threshold", ransac_threshold))
        pSingleview_recognizer->set_cg_ransac_threshold(ransac_threshold);

    if(n_->getParam ( "cg_dist_for_clutter_factor", dist_for_clutter_factor))
        pSingleview_recognizer->set_cg_dist_for_clutter_factor(dist_for_clutter_factor);

    if(n_->getParam ( "cg_max_taken", max_taken))
        pSingleview_recognizer->set_cg_max_taken(max_taken);

    if(n_->getParam ( "cg_max_time_for_cliques_computation", max_time_for_cliques_computation))
        pSingleview_recognizer->set_cg_max_time_for_cliques_computation(max_time_for_cliques_computation);

    if(n_->getParam ( "cg_dot_distance", dot_distance))
        pSingleview_recognizer->set_cg_dot_distance(dot_distance);


    if(n_->getParam ( "hv_resolution", resolution))
        pSingleview_recognizer->set_hv_resolution(resolution);

    if(n_->getParam ( "hv_inlier_threshold", inlier_threshold))
        pSingleview_recognizer->set_hv_inlier_threshold(inlier_threshold);

    if(n_->getParam ( "hv_radius_clutter", radius_clutter))
        pSingleview_recognizer->set_hv_radius_clutter(radius_clutter);

    if(n_->getParam ( "hv_regularizer", regularizer))
        pSingleview_recognizer->set_hv_regularizer(regularizer);

    if(n_->getParam ( "hv_clutter_regularizer", clutter_regularizer))
        pSingleview_recognizer->set_hv_clutter_regularizer(clutter_regularizer);

    if(n_->getParam ( "hv_occlusion_threshold", occlusion_threshold))
        pSingleview_recognizer->set_hv_occlusion_threshold(occlusion_threshold);

    if(n_->getParam ( "hv_optimizer_type", optimizer_type))
        pSingleview_recognizer->set_hv_optimizer_type(optimizer_type);

    if(n_->getParam ( "hv_color_sigma_l", color_sigma_l))
        pSingleview_recognizer->set_hv_color_sigma_L(color_sigma_l);

    if(n_->getParam ( "hv_color_sigma_ab", color_sigma_ab))
        pSingleview_recognizer->set_hv_color_sigma_AB(color_sigma_ab);



    pSingleview_recognizer->initialize();


    worldRepresentation myWorld;
    myWorld.setModels_dir(models_dir);
    myWorld.setPSingleview_recognizer(pSingleview_recognizer);
    myWorld.setSift(sift);

    if(n_->getParam ( "opt_type", opt_type))
        myWorld.setOpt_type(opt_type);

    if(n_->getParam ( "chop_z", chop_at_z))
        myWorld.setChop_at_z(chop_at_z);

    if(n_->getParam ( "scene_to_scene", scene_to_scene))
        myWorld.set_scene_to_scene(scene_to_scene);

    if(n_->getParam ( "visualize_output", visualize_output))
        myWorld.set_visualize_output(visualize_output);

    if(n_->getParam ( "max_vertices_in_graph", max_vertices_in_graph))
        myWorld.set_max_vertices_in_graph(max_vertices_in_graph);

    if(n_->getParam ( "distance_keypoints_get_discarded", distance_keypoints_get_discarded))
        myWorld.set_distance_keypoints_get_discarded(distance_keypoints_get_discarded);

    //client_ = n_->serviceClient<recognition_srv_definitions::recognize> ( "/recognition_service/mp_recognition" );
    ros::ServiceServer ros_mv_rec_server;
    ros_mv_rec_server = n_->advertiseService("multiview_recognotion_servcice", &worldRepresentation::recognize, &myWorld);

    ros::Publisher vis_pc_pub;
    vis_pc_pub = n_->advertise<sensor_msgs::PointCloud2>( "test", 0 );
    myWorld.set_vis_pc_pub(vis_pc_pub);

    ROS_INFO("Multiview object recognizer is ready to get service calls.");
    ros::spin();

    return 0;
}
