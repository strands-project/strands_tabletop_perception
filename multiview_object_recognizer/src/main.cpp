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
    int opt_type;
    double chop_at_z;
    std::string training_dir_sift, training_dir_shot, sift_structure, training_dir_ourcvfh;
    bool do_sift, do_ourcvfh, do_shot, ignore_color;

    ros::init ( argc, argv, "multiview_object_recognizer_node" );
    n_.reset( new ros::NodeHandle ( "~" ) );

    if ( ! n_->getParam ( "models_dir", models_dir ))
    {
        std::cout << "No models_dir specified. " << std::endl;
    }

    n_->getParam ( "visualize_output", visualize_output);
    n_->getParam ( "scene_to_scene", scene_to_scene);
    n_->getParam ( "icp_iterations", icp_iter);
    n_->getParam ( "opt_type", opt_type);
    n_->getParam ( "chop_z", chop_at_z);

    n_->getParam ( "training_dir_sift", training_dir_sift);
    n_->getParam ( "training_dir_shot", training_dir_shot);
    n_->getParam ( "recognizer_structure_sift", sift_structure);
    n_->getParam ( "training_dir_ourcvfh", training_dir_ourcvfh);
    n_->getParam ( "do_sift", do_sift);
    n_->getParam ( "do_shot", do_shot);
    n_->getParam ( "do_ourcvfh", do_ourcvfh);
    n_->getParam ( "ignore_color", ignore_color);

    std::cout << chop_at_z << ", " << ignore_color << std::endl;
    if (models_dir.compare ("") == 0)
    {
        PCL_ERROR ("Set -models_dir option in the command line, ABORTING");
        return -1;
    }

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
    pSingleview_recognizer->setChop_at_z(chop_at_z);
    pSingleview_recognizer->setDo_sift(do_sift);
    if(do_sift)
    {
        pSingleview_recognizer->setSift(sift);
    }
    pSingleview_recognizer->setDo_shot(do_shot);
    pSingleview_recognizer->setDo_ourcvfh(do_ourcvfh);
    pSingleview_recognizer->setIgnore_color(ignore_color);
    pSingleview_recognizer->setIcp_iterations(icp_iter);
    pSingleview_recognizer->initialize();


    worldRepresentation myWorld;
    myWorld.setModels_dir(models_dir);
    myWorld.setVisualize_output(visualize_output);
    myWorld.setIcp_iter(icp_iter);
    myWorld.setOpt_type(opt_type);
    myWorld.setChop_at_z(chop_at_z);
    myWorld.setPSingleview_recognizer(pSingleview_recognizer);
    myWorld.setSift(sift);
    myWorld.set_scene_to_scene(scene_to_scene);

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
