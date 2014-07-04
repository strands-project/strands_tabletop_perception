#include <iostream>
#include <boost/filesystem.hpp>
#include <pcl_conversions.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>
#include <ros/ros.h>
#include <faat_pcl/utils/filesystem_utils.h>
#include <faat_pcl/utils/pcl_visualization_utils.h>
#include <vector>
#include <sstream>
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
    ros::init(argc, argv, "multiview_object_recognizer_test_node");
    ros::NodeHandle *n;
    n = new ros::NodeHandle("~");
    std::string scenes_dir, camera_topic;
    bool visualize_output;
    ros::ServiceClient mv_recognition_client = n->serviceClient<recognition_srv_definitions::multiview_recognize>("/multiview_object_recognizer_node/multiview_recognotion_servcice");
    recognition_srv_definitions::multiview_recognize srv;


    if(!n->getParam("scenes_dir", scenes_dir))
        ROS_ERROR("No scenes directory given (arg \"scenes_dir\"). ");
    if(!n->getParam("visualize_output", visualize_output))
        visualize_output = true;
    if(!n->getParam("topic", camera_topic))
        camera_topic = "/camera/depth_registered/points";

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
        for (size_t file_id=0; file_id < files_intern.size(); file_id++)
        {
            std::stringstream full_file_name;
            full_file_name << scenes_dir << "/" << files_intern[file_id];
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr pScene (new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::io::loadPCDFile(full_file_name.str(), *pScene);

            sensor_msgs::PointCloud2  pc2;
            pcl::toROSMsg (*pScene, pc2);
            srv.request.cloud = pc2;
            srv.request.scene_name.data = scenes_dir;
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

                        Eigen::Matrix4f tt;
                        tt.setIdentity ( 4,4 );

                        tt ( 0,3 ) = srv.response.transforms[i].translation.x;
                        tt ( 1,3 ) = srv.response.transforms[i].translation.y;
                        tt ( 2,3 ) = srv.response.transforms[i].translation.z;
                        Eigen::Quaternionf q ( srv.response.transforms[i].rotation.w,
                                               srv.response.transforms[i].rotation.x,
                                               srv.response.transforms[i].rotation.y,
                                               srv.response.transforms[i].rotation.z );

                        Eigen::Matrix3f rot = q.toRotationMatrix();
                        tt.block<3,3> ( 0,0 ) = rot;

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

        if(visualize_output)
        {
            pcl::visualization::PCLVisualizer::Ptr vis (new pcl::visualization::PCLVisualizer("vis"));
            std::vector<int> viewportNr = faat_pcl::utils::visualization_framework (vis, files_intern.size(), 1);
            //        vis->setWindowName ("Hypotheses transformation and verification for multiple views");
            //        for (std::vector<Vertex>::iterator it_vrtx = vertices_v.begin (); it_vrtx != vertices_v.end (); ++it_vrtx)
            //        {
            //            std::stringstream cloud_name_tmp;
            //            cloud_name_tmp << "scene_cloud_" << grph_final[*it_vrtx].scene_filename;
            //            pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> handler_rgb (grph_final[*it_vrtx].pScenePCl);
            //            vis->addPointCloud<pcl::PointXYZRGB> (grph_final[*it_vrtx].pScenePCl, handler_rgb, cloud_name_tmp.str (),
            //                                                  viewportNr[boost::get (vertex_index, grph_final, *it_vrtx) * SUBWINDOWS_PER_VIEW_HT_FROM_FILE + 0]);

            //            for (size_t hypVec_id = 0; hypVec_id < grph_final[*it_vrtx].hypothesis.size (); hypVec_id++)
            //            {

            //                PointInTPtr pHypothesisPCl (new pcl::PointCloud<pcl::PointXYZRGB>);
            //                PointInTPtr pHypothesisPCl_vx (new pcl::PointCloud<pcl::PointXYZRGB>);
            //                PointInTPtr pHypothesisAlignedPCl (new pcl::PointCloud<pcl::PointXYZRGB>);
            //                pcl::io::loadPCDFile (grph_final[*it_vrtx].hypothesis[hypVec_id].model_id_, *pHypothesisPCl);

            //                pcl::VoxelGrid<pcl::PointXYZRGB> sor;
            //                float leaf = 0.0025f;
            //                sor.setLeafSize (leaf, leaf, leaf);
            //                sor.setInputCloud (pHypothesisPCl);
            //                sor.filter (*pHypothesisPCl_vx);

            //                pcl::transformPointCloud (*pHypothesisPCl_vx, *pHypothesisAlignedPCl, grph_final[*it_vrtx].hypothesis[hypVec_id].transform_);
            //                pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> handler_rgb_verified (pHypothesisAlignedPCl);
            //                std::stringstream basename;
            //                basename << "Hypothesis_model_" << grph_final[*it_vrtx].hypothesis[hypVec_id].model_id_ << "__forScene_" << grph_final[*it_vrtx].scene_filename
            //                         << "__origin_" << grph_final[*it_vrtx].hypothesis[hypVec_id].origin_ << "__with_vector_id_" << hypVec_id;

            //                if(grph_final[*it_vrtx].hypothesis[hypVec_id].origin_.compare(grph_final[*it_vrtx].scene_filename) == 0)	//--show-hypotheses-from-single-view
            //                {
            //                    std::stringstream name;
            //                    name << "Single_View_" << basename.str();
            //                    vis->addPointCloud<pcl::PointXYZRGB> (pHypothesisAlignedPCl, handler_rgb_verified, name.str (),
            //                                                          viewportNr[boost::get (vertex_index, grph_final, *it_vrtx) * SUBWINDOWS_PER_VIEW_HT_FROM_FILE + 1]);
            //                }

            //                /*std::stringstream name;
            //      name << "After_Hyp_Extension_" << basename.str();
            //      vis->addPointCloud<pcl::PointXYZRGB> (pHypothesisAlignedPCl, handler_rgb_verified, name.str (),
            //                          viewportNr[boost::get (vertex_index, grph_final, *it_vrtx) * SUBWINDOWS_PER_VIEW_HT_FROM_FILE + 2]);*/


            //                if(grph_final[*it_vrtx].hypothesis[hypVec_id].verified_)	//--show-verified-hypotheses
            //                {
            //                    //std::cout << grph_final[*it_vrtx].hypothesis[hypVec_id].transform_ << std::endl;
            //                    std::stringstream name;
            //                    name << "After_Hyp_Extension_and_Verification_" << basename.str();
            //                    vis->addPointCloud<pcl::PointXYZRGB> (pHypothesisAlignedPCl, handler_rgb_verified, name.str (),
            //                                                          viewportNr[boost::get (vertex_index, grph_final, *it_vrtx) * SUBWINDOWS_PER_VIEW_HT_FROM_FILE + 3]);
            //                }
            //            }
            //        }
            vis->setBackgroundColor(1,1,1);
            vis->resetCamera();
            //vis->setFullScreen(true);
            vis->spin ();
            vis->getInteractorStyle()->saveScreenshot("multiview.png");
        }
    }
    ros::   spin();
    return 0;

}
