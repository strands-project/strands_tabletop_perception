#include "my_classifier.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <iostream>
#include <ros/ros.h>
#include <v4r/svm/svmWrapper.h>

int
main (int argc, char ** argv)
{
    bool force_retrain_;
    std::string models_dir, trained_dir, test_filename, test_dir;
    int features_2d, features_3d;
    ros::init(argc, argv, "generic_object_classifier");
    ros::NodeHandle *n_;
    n_ = new ros::NodeHandle ( "~" );

    if ( ! n_->getParam ( "test_filename", test_filename ))
    {
        std::cout << "No test filename set" << std::endl;
    }

    if ( ! n_->getParam ( "trained_dir", trained_dir ))
    {
        std::cout << "No trained dir set. " << std::endl;
    }

    if ( ! n_->getParam ( "training_dir", models_dir ) )
    {
        std::cout << "No training dir set. " << std::endl;
    }

    if ( ! n_->getParam ( "test_dir", test_dir ) )
    {
        std::cout << "No test dir set. " << std::endl;
    }

    if ( ! n_->getParam ( "force_retrain", force_retrain_ ) )
    {
        std::cout << "Force retrain not specified as argument. " << std::endl;
    }

    if ( ! n_->getParam ( "features_2d", features_2d ) )
    {
        std::cerr << "No 2D features selected. " << std::endl;
        features_2d = 0;
    }

    if ( ! n_->getParam ( "features_3d", features_3d ) )
    {
        std::cerr << "No 3D features selected. " << std::endl;
        features_3d = 0;
    }

    std::cout << test_filename << trained_dir << models_dir
                 << test_dir << force_retrain_ << std::endl;

    pcl::PointCloud<PointT>::Ptr pCloud, pSegmentedCloud;
    pCloud.reset(new pcl::PointCloud<PointT>());
    pSegmentedCloud.reset(new pcl::PointCloud<PointT>());

    MyClassifier classifier;
    classifier.setForceRetrain(force_retrain_);
    classifier.setTestDir(test_dir);
    classifier.setTrainedDir(trained_dir);
    classifier.setTrainingDir(models_dir);
    classifier.setFeatures2D(static_cast<size_t>( features_2d ));
    classifier.setFeatures3D(static_cast<size_t>( features_3d ));
    classifier.init ();
    classifier.trainClassifier();
    classifier.testClassifier();
    //classifier.classify();

    /*pcl::visualization::PCLVisualizer::Ptr vis;
    vis.reset(new pcl::visualization::PCLVisualizer("classifier visualization"));
    pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb_handler (pSegmentedCloud);
    vis->addPointCloud<PointT> (pSegmentedCloud, rgb_handler, "classified_pcl");
    vis->spin();*/
    return 0;
}
