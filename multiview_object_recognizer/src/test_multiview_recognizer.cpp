#include <iostream>
#include <boost/filesystem.hpp>
#include <fstream>
#include "ros/ros.h"
#include <vector>
#include <sstream>
#include <string>
#include <faat_pcl/utils/filesystem_utils.h>
#include <pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>
#include <recognition_srv_definitions/multiview_recognize.h>
#include "scitos_apps_msgs/action_buttons.h"


void joyCallback ( const scitos_apps_msgs::action_buttons& msg );
void kinectCallback ( const sensor_msgs::PointCloud2& msg );

ros::Subscriber sub_joy_, sub_pc_;
//namespace bf = boost::filesystem;

void joyCallback ( const scitos_apps_msgs::action_buttons& msg )
{
    ROS_INFO ( "Button pressed." );
    //recognize ( *current_cloud_ );
}

void kinectCallback ( const sensor_msgs::PointCloud2& msg )
{
    //    current_cloud_mutex_.lock();
    //    pcl::fromROSMsg ( msg, *current_cloud_ );
    //    current_cloud_mutex_.unlock();
}

int
main (int argc, char **argv)
{
    const std::string transform_prefix_ = "transformation_";
    ros::init(argc, argv, "multiview_object_recognizer_test_node");
    ros::NodeHandle *n;
    n = new ros::NodeHandle("~");
    std::string dataset_path, sequence_name, camera_topic;
    ros::ServiceClient mv_recognition_client = n->serviceClient<recognition_srv_definitions::multiview_recognize>("/multiview_object_recognizer_node/multiview_recognotion_servcice");
    recognition_srv_definitions::multiview_recognize srv;

    if(!n->getParam("dataset_path", dataset_path))
        ROS_ERROR("No dataset path given (arg \"dataset_path\"). ");

    if(!n->getParam("sequence_name", sequence_name))
        ROS_ERROR("No sequence name given (arg \"sequence_name\"). ");

    if(!n->getParam("topic", camera_topic))
        camera_topic = "/camera/depth_registered/points";

    std::stringstream scenes_dir_ss;
    scenes_dir_ss << dataset_path << "/" << sequence_name;
    std::string scenes_dir = scenes_dir_ss.str();

    boost::filesystem::path scenes_dir_bf = scenes_dir;
    if (!boost::filesystem::exists (scenes_dir_bf)) //no hypothesis exist yet --> create
    {

        ROS_ERROR("Scene directory is emtpy or does not exist. ");
        //sub_pc_  = n.subscribe ( camera_topic, 1, &multiviewGraph::kinectCallback, this );
        //sub_joy_ = n.subscribe ( "/teleop_joystick/action_buttons", 1, &multiviewGraph::joyCallback, this );
        ROS_INFO ( "Start online recognition of topic %s by pressing a button...", camera_topic.c_str() );
        //return -1;
    }
    else
    {
        std::vector < std::string > files_intern;
        std::string start = "";
        std::string ext = std::string ("pcd");
        faat_pcl::utils::getFilesInDirectory (scenes_dir_bf, start, files_intern, ext);

        ros::Publisher vis_pc_pub_ = n->advertise<sensor_msgs::PointCloud2>( "multiview_point_cloud_input", 0 );

        for (size_t file_id=0; file_id < files_intern.size(); file_id++)
        {
            std::stringstream full_file_name;
            full_file_name << scenes_dir << "/" << files_intern[file_id];
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr pScene (new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::io::loadPCDFile(full_file_name.str(), *pScene);

//            pcl::visualization::PCLVisualizer::Ptr vis (new pcl::visualization::PCLVisualizer("vis"));
//            pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> handler_rgb (pScene);
//            vis->addPointCloud<pcl::PointXYZRGB> (pScene, handler_rgb, "scnene");
//            vis->spin();

            sensor_msgs::PointCloud2  pc2;
            pcl::toROSMsg (*pScene, pc2);
            pc2.header.frame_id = "camera_link";
            pc2.header.stamp = ros::Time::now();
            pc2.is_dense = false;
            vis_pc_pub_.publish(pc2);
            ros::spinOnce(); // Why does the point cloud not get published immediately?

            srv.request.cloud = pc2;
            srv.request.scene_name.data = sequence_name;
            srv.request.view_name.data = files_intern[file_id].substr(0, files_intern[file_id].length() - ext.length() - 1);
            srv.request.transform.clear();

            std::stringstream transform_ss;
            transform_ss << scenes_dir << "/" << transform_prefix_ << files_intern[file_id].substr(0, files_intern[file_id].length() - ext.length()) << "txt";
            std::cout << "Checking if path " << transform_ss.str() << " for transform exists. " << std::endl;
            if ( boost::filesystem::exists( transform_ss.str() ) )
            {
                std::cout << "File exists." << std::endl;
                std::ifstream is(transform_ss.str());
                std::istream_iterator<double> start(is), end;
                std::vector<double> numbers(start, end);
                std::cout << "Read " << numbers.size() << " numbers" << std::endl;

                // print the numbers to stdout
                std::cout << "Transform to world coordinate system: " << std::endl;
                for(size_t i=0; i<numbers.size(); i++)
                {
                    std::cout << numbers[i] << " ";
                    srv.request.transform.push_back(numbers[i]);
                }
                std::cout << std::endl;

            }
            else
            {
                std::cout << "File does not exist. Using it without world transform." << std::endl;
            }


            if (mv_recognition_client.call(srv))
            {
                if ( srv.response.ids.size() == 0 )
                {
                    ROS_INFO ( "I didn't detect any object in the current scene." );
                }
                else
                {
                    for ( size_t i=0; i < srv.response.ids.size(); i++ )
                    {
                        std_msgs::String object_id = ( std_msgs::String ) srv.response.ids[i];
                        ROS_INFO ( "I detected object %s in the scene.", object_id.data.c_str() );
                        //model_ids.push_back ( srv.response.ids[i].data );

//                        Eigen::Matrix4f tt;
//                        tt.setIdentity ( 4,4 );

//                        tt ( 0,3 ) = srv.response.transforms[i].translation.x;
//                        tt ( 1,3 ) = srv.response.transforms[i].translation.y;
//                        tt ( 2,3 ) = srv.response.transforms[i].translation.z;
//                        Eigen::Quaternionf q ( srv.response.transforms[i].rotation.w,
//                                               srv.response.transforms[i].rotation.x,
//                                               srv.response.transforms[i].rotation.y,
//                                               srv.response.transforms[i].rotation.z );

//                        Eigen::Matrix3f rot = q.toRotationMatrix();
//                        tt.block<3,3> ( 0,0 ) = rot;

                        //transforms.push_back ( tt );

                        //                        std::stringstream model_name;
                        //                        model_name << models_dir_ << srv.response.ids[i].data;
                        //                        Hypothesis hypothesis ( model_name.str(), tt, grph_[vrtx].scene_filename_, false );
                        //                        grph_[vrtx].hypothesis.push_back ( hypothesis );
                    }
                }
            }
            else
            {
                ROS_ERROR("Failed to call multiview recognition server.");
                return -1;
            }
        }
    }
    ros::   spin();
    return 0;

}
