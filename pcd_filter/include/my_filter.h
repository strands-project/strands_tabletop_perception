#ifndef MY_FILTER_H
#define MY_FILTER_H

#include <vector>
#include <ros/ros.h>
#include <pcl/common/common.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/io.h>

class TableTopFilter
{
private:

    std::string input_dir_;
    bool visualize_;
    double chop_z_max_;
    ros::NodeHandle *n_;
    std::string indices_prefix_;
    bool force_refilter_;


public:

    TableTopFilter()
    {
        indices_prefix_ = "object_indices_";
        force_refilter_ = true;
        visualize_ = true;
    }

    void init(int argc, char ** argv)
    {
        ros::init(argc, argv, "pcd_filter_node");
        n_ = new ros::NodeHandle ( "~" );
        if(! n_->getParam ( "input_dir", input_dir_ ))
            std::cerr << "No input dir set. " << std::endl;

        if(! n_->getParam ( "chop_z_max", chop_z_max_ ))
        {
            chop_z_max_ = 1.2f;
            std::cerr << "No maximum z-distance set. Setting it to default value " << chop_z_max_ << ". " << std::endl;
        }

        n_->getParam ( "visualize", visualize_ );
        n_->getParam ( "force_refilter", force_refilter_ );
    }

    void filterAndWriteToFile(const std::string path);
    void filterAndWriteToFileRecursive(const std::string path);


    std::string getInputDir()
    {
        return input_dir_;
    }
};


#endif
