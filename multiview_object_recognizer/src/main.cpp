#include "singleview_object_recognizer.h"
#include "world_representation.h"

#include "ros/ros.h"
#include <iostream>

#include <pcl/io/pcd_io.h>
ros::ServiceClient client_;
boost::shared_ptr<ros::NodeHandle> n_;
ros::ServiceServer ros_mv_rec_server_;
ros::Publisher vis_pc_pub_;

int main (int argc, char **argv)
{
    std::string models_dir;
    bool visualize_output;
    bool go_3d;
    int icp_iter;
    int opt_type;
    std::string gt_or_ouput_dir;
    double chop_at_z;
    int mv_keypoints;
    std::string training_dir_sift, sift_structure, training_dir_ourcvfh;
    bool do_sift, do_ourcvfh, ignore_color;

    ros::init ( argc, argv, "multiview_object_recognizer_node" );
    n_.reset( new ros::NodeHandle ( "~" ) );

    if ( ! n_->getParam ( "models_dir", models_dir ))
    {
        std::cout << "No models_dir specified. " << std::endl;
    }

    n_->getParam ( "visualize_output", visualize_output);
    n_->getParam ( "go_3d", go_3d);
    n_->getParam ( "gt_or_output_dir", gt_or_ouput_dir);
    n_->getParam ( "icp_iterations", icp_iter);
    n_->getParam ( "mv_keypoints", mv_keypoints);
    n_->getParam ( "opt_type", opt_type);
    n_->getParam ( "chop_z", chop_at_z);

    n_->getParam ( "training_dir_sift", training_dir_sift);
    n_->getParam ( "recognizer_structure_sift", sift_structure);
    n_->getParam ( "training_dir_ourcvfh", training_dir_ourcvfh);
    n_->getParam ( "do_sift", do_sift);
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
    pSingleview_recognizer->setModels_dir(models_dir);
    pSingleview_recognizer->setSift_structure(sift_structure);
    pSingleview_recognizer->setChop_at_z(chop_at_z);
    pSingleview_recognizer->setDo_sift(do_sift);
    if(do_sift)
    {
        pSingleview_recognizer->setSift(sift);
    }
    pSingleview_recognizer->setDo_ourcvfh(do_ourcvfh);
    pSingleview_recognizer->setIgnore_color(ignore_color);
    pSingleview_recognizer->setIcp_iterations(icp_iter);
    pSingleview_recognizer->initialize();



    worldRepresentation myWorld;
    myWorld.setModels_dir(models_dir);
    myWorld.setVisualize_output(visualize_output);
    myWorld.setGo_3d(go_3d);
    myWorld.setGt_or_ouput_dir(gt_or_ouput_dir);
    myWorld.setIcp_iter(icp_iter);
    myWorld.setMv_keypoints(mv_keypoints);
    myWorld.setOpt_type(opt_type);
    myWorld.setChop_at_z(chop_at_z);
    myWorld.setPSingleview_recognizer(pSingleview_recognizer);
    myWorld.setSift(sift);

    //client_ = n_->serviceClient<recognition_srv_definitions::recognize> ( "/recognition_service/mp_recognition" );
    ros_mv_rec_server_ = n_->advertiseService("multiview_recognotion_servcice", &worldRepresentation::recognize, &myWorld);

    vis_pc_pub_ = n_->advertise<sensor_msgs::PointCloud2>( "test", 0 );

    ROS_INFO("Multiview object recognizer is ready to get service calls.");
    ros::spin();

    return 0;
}
