/*
 * shape_simple_classifier_node.cpp
 *
 *  Created on: Sep 7, 2013
 *      Author: aitor
 */

#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include <pcl/common/common.h>
#include <pcl/console/parse.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "classifier_srv_definitions/segment_and_classify.h"
#include "classifier_srv_definitions/mv_classify.h"
#include "object_perception_msgs/classification.h"
#include <boost/algorithm/string/predicate.hpp>
#include <boost/lexical_cast.hpp>
#include <Eigen/Eigenvalues>
#include "std_msgs/String.h"
#include "std_msgs/Int32MultiArray.h"
#include "geometry_msgs/Point32.h"
#include "geometry_msgs/Transform.h"
#include "pcl/common/transforms.h"
#include <pcl/octree/octree_pointcloud_pointvector.h>
#include <pcl/segmentation/extract_clusters.h>
#include <algorithm>
#include <numeric>

class MVObjectClassifier
{
  private:
    typedef pcl::PointXYZRGB PointT;
    typedef pcl::PointXYZL OctreePointT;
    boost::shared_ptr<pcl::octree::OctreePointCloudPointVector<OctreePointT> > octree_;
    pcl::PointCloud<OctreePointT>::Ptr octree_cloud_full_;

    double chop_at_z_;
    std::string session_id_;
    std::vector<pcl::PointCloud<PointT>::Ptr> frames_;
    std::vector<Eigen::Matrix4f> poses_to_global_;

    ros::ServiceServer add_cloud_service_;
    ros::NodeHandle *n_;
    float octree_resolution_;

    std::vector< object_perception_msgs::classification> class_results_ros_all_views_;

    boost::shared_ptr<pcl::visualization::PCLVisualizer> vis_;
    int v1_,v2_, v3_;
    std::vector<std::map<std::string, float > > octree_voxels_class_probabilities_;
    bool visualize_output_;

    Eigen::Matrix4f GeometryMsgToMatrix4f(geometry_msgs::Transform & tt)
    {
        Eigen::Matrix4f trans = Eigen::Matrix4f::Identity();
        trans(0,3) = tt.translation.x;
        trans(1,3) = tt.translation.y;
        trans(2,3) = tt.translation.z;

        Eigen::Quaternionf q(tt.rotation.w,tt.rotation.x,tt.rotation.y,tt.rotation.z);
        trans.block<3,3>(0,0) = q.toRotationMatrix();
        return trans;
    }

    void reset()
    {
        frames_.clear();

        if(visualize_output_)
        {
            vis_->removeAllPointClouds();
        }

        octree_cloud_full_.reset(new pcl::PointCloud<OctreePointT>);
        class_results_ros_all_views_.clear();

        octree_.reset(new pcl::octree::OctreePointCloudPointVector<OctreePointT>(octree_resolution_));
        octree_->setInputCloud(octree_cloud_full_);
    }

    //
    // Compute a voxelgrided version of the cloud
    //      => Each voxel is associated with a class-probability list based on the sv classification results
    //          ... Weighted sum of points falling into the voxel (ignore all but one if coming from same classification result)

    void
    getPointCloudFromOctree(pcl::PointCloud<OctreePointT>::Ptr & output)
    {
        unsigned int leaf_node_counter = 0;
        pcl::octree::OctreePointCloudPointVector<OctreePointT>::LeafNodeIterator it2;
        const pcl::octree::OctreePointCloudPointVector<OctreePointT>::LeafNodeIterator it2_end = octree_->leaf_end();

        std::cout << "size octree input cloud:" << octree_->getInputCloud()->points.size() << std::endl;
        output->points.resize(octree_->getInputCloud()->points.size());
        octree_voxels_class_probabilities_.resize(octree_->getInputCloud()->points.size());

        int kept = 0;
        for (it2 = octree_->leaf_begin(); it2 != it2_end; ++it2)
        {
            ++leaf_node_counter;
            pcl::octree::OctreeContainerPointIndices& container = it2.getLeafContainer();
            // add points from leaf node to indexVector
            std::vector<int> indexVector;
            container.getPointIndices (indexVector);

            OctreePointT p;
            p.getVector3fMap() = Eigen::Vector3f::Zero();

            std::vector<bool> label_used_for_voxel(class_results_ros_all_views_.size(), false);
            std::map<std::string, std::vector<float> > probabilites_per_class_for_voxel;

            int used = 0;
            for(size_t k=0; k < indexVector.size(); k++)
            {

                /*std::cout << class_results_ros_[i].class_type[kk].data <<
                             " [" << class_results_ros_[i].confidence[kk] << "]" << std::endl*/

                //assert(indexVector[k] < octree_->getInputCloud()->points.size());
                uint32_t label = octree_->getInputCloud()->points[indexVector[k]].label;
                if(!label_used_for_voxel[label])
                {
                    assert(label < class_results_ros_all_views_.size());
                    p.getVector3fMap() = p.getVector3fMap() +  octree_->getInputCloud()->points[indexVector[k]].getVector3fMap();
                    used++;
                    label_used_for_voxel[label] = true;
                    std::map<std::string, std::vector<float> >::iterator it;
                    for(size_t kk=0; kk < class_results_ros_all_views_[label].class_type.size(); kk++)
                    {
                        std::string class_type = class_results_ros_all_views_[label].class_type[kk].data;
                        float conf = class_results_ros_all_views_[label].confidence[kk];
                        it = probabilites_per_class_for_voxel.find(class_type);
                        if(it == probabilites_per_class_for_voxel.end())
                        {
                            std::vector<float> confs;
                            confs.push_back(conf);
                            probabilites_per_class_for_voxel.insert(std::make_pair(class_type, confs));
                        }
                        else
                        {
                            it->second.push_back(conf);
                        }
                    }
                }
                else
                {
                    //ignore
                }
            }

            if(used > 0)
            {
                p.getVector3fMap() = p.getVector3fMap() / used;
            }
            else
            {
                continue;
            }

            //compute weighted sum
            std::map<std::string, std::vector<float> >::iterator it;
            float total_sum = 0;
            for(it = probabilites_per_class_for_voxel.begin(); it != probabilites_per_class_for_voxel.end(); it++)
            {
                float sum = 0;
                for(size_t k=0; k < it->second.size(); k++)
                {
                    sum += it->second[k];
                }

                octree_voxels_class_probabilities_[kept].insert(std::make_pair(it->first, sum));
                total_sum += sum;
            }

            //normalize
            std::map<std::string, float >::iterator it_main;
            for(it_main = octree_voxels_class_probabilities_[kept].begin();
                it_main != octree_voxels_class_probabilities_[kept].end();
                it_main++)
            {
                it_main->second /= total_sum;
            }

            //assert(kept < output->points.size());
            output->points[kept] = p;

            kept++;
        }

        output->points.resize(kept);
        output->width = kept;
        output->height = 1;
        output->is_dense = true;
        octree_voxels_class_probabilities_.resize(kept);
    }

    bool add_cloud(classifier_srv_definitions::mv_classify::Request & req,
                   classifier_srv_definitions::mv_classify::Response & response)
    {

        std::cout << "add cloud called" << " " << frames_.size() << std::endl;

        //Compare session ids and call reset if different
        std::string current_session_id = req.session_id.data;
        if(current_session_id.compare(session_id_) != 0)
        {
            reset();
            session_id_ = current_session_id;
        }

        pcl::PointCloud<PointT>::Ptr frame(new pcl::PointCloud<PointT>);
        pcl::fromROSMsg(req.cloud, *frame);

        //classify frame and get single view results, service call to sv_classifier
        ros::ServiceClient segAndClassifierClient = n_->serviceClient<classifier_srv_definitions::segment_and_classify>("/classifier_service/segment_and_classify");
        classifier_srv_definitions::segment_and_classify srv;
        srv.request.cloud = req.cloud;
        std::vector< std_msgs::Int32MultiArray> cluster_indices_ros_;
        std::vector< object_perception_msgs::classification> class_results_ros_;

        if (segAndClassifierClient.call(srv))
        {
            class_results_ros_ = srv.response.class_results;
            cluster_indices_ros_ = srv.response.clusters_indices;
        }
        else
        {
            ROS_ERROR("Failed to call SINGLE VIEW segmentation_and_classifier service.");
            return false;
        }

        frames_.push_back(frame);

        Eigen::Matrix4f t = GeometryMsgToMatrix4f(req.transform);
        poses_to_global_.push_back(t);

        //for each frame, each point (only in the returned clusters) votes on a certain voxel

        for(size_t i=0; i < cluster_indices_ros_.size(); i++)
        {
            pcl::PointCloud<PointT>::Ptr processed(new pcl::PointCloud<PointT>);
            pcl::PointIndices cluster_indices;
            cluster_indices.indices = cluster_indices_ros_[i].data;
            pcl::copyPointCloud(*frame, cluster_indices, *processed);
            pcl::transformPointCloud(*processed, *processed, t);

            for(size_t k=0; k < processed->points.size(); k++)
            {
                OctreePointT p;
                p.getVector3fMap() = processed->points[k].getVector3fMap();
                p.label = class_results_ros_all_views_.size();
                octree_->addPointToCloud(p, octree_cloud_full_);
            }

            class_results_ros_all_views_.push_back(class_results_ros_[i]);
        }

        if(frames_.size() > 1)
        {

            //TODO: Fix this
            octree_voxels_class_probabilities_.clear();
            pcl::PointCloud<OctreePointT>::Ptr cloud_octree(new pcl::PointCloud<OctreePointT>);
            getPointCloudFromOctree(cloud_octree);

            //segment from octree to generate results
            std::vector<pcl::PointIndices> cluster_indices;
            pcl::EuclideanClusterExtraction<OctreePointT> ec;
            ec.setClusterTolerance (octree_resolution_ * 2.f);
            ec.setMinClusterSize (100);
            ec.setInputCloud (cloud_octree);
            ec.extract (cluster_indices);

            std::cout << "number of clusters:" << cluster_indices.size() << std::endl;

            //once we have segmented, we need to compute the final probabilities for each cluster
            response.class_results.resize(cluster_indices.size());
            response.centroid.resize(cluster_indices.size());

            for(size_t i=0; i < cluster_indices.size(); i++)
            {

                Eigen::Vector4f centroid;
                pcl::compute3DCentroid(*cloud_octree, cluster_indices[i], centroid);

                geometry_msgs::Point32 centroid_ros;
                centroid_ros.x = centroid[0];
                centroid_ros.y = centroid[1];
                centroid_ros.z = centroid[2];
                response.centroid[i] = centroid_ros;

                std::map<std::string, std::vector<float> > cluster_class_probabilities;
                std::map<std::string, std::vector<float> >::iterator it;

                for(size_t k=0; k < cluster_indices[i].indices.size(); k++)
                {
                    std::map<std::string, float>::iterator it_main;
                    for(it_main = octree_voxels_class_probabilities_[cluster_indices[i].indices[k]].begin();
                        it_main != octree_voxels_class_probabilities_[cluster_indices[i].indices[k]].end();
                        it_main++)
                    {
                        //std::cout << it_main->first << " --- " << it_main->second << std::endl;
                        it = cluster_class_probabilities.find(it_main->first);
                        if(it == cluster_class_probabilities.end())
                        {
                            std::vector<float> conffs;
                            conffs.push_back(it_main->second);
                            cluster_class_probabilities.insert(std::make_pair(it_main->first, conffs));
                        }
                        else
                        {
                            it->second.push_back(it_main->second);
                        }
                    }
                }

                //std::cout << std::endl;
                std::map<std::string, std::vector<float> >::iterator it2;
                float cluster_size = cluster_indices[i].indices.size();
                for(it2 = cluster_class_probabilities.begin(); it2 != cluster_class_probabilities.end(); it2++)
                {

                    float sum = std::accumulate(it2->second.begin(), it2->second.end(), 0.f);
                    float conf = sum / cluster_size;

                    std_msgs::String str_tmp;
                    str_tmp.data = it2->first;
                    response.class_results[i].class_type.push_back(str_tmp);
                    response.class_results[i].confidence.push_back(conf);
                }

            }

            if(visualize_output_)
            {

                std::cout << "Number of clusters:" << response.class_results.size() << std::endl;
                for(size_t i=0; i < response.class_results.size(); i++)
                {
                    std::cout << response.class_results[i].class_type.size() << std::endl;
                }

                for(size_t i=0; i < response.class_results.size(); i++)
                {
                    for (size_t kk = 0; kk < response.class_results[i].class_type.size(); kk++)
                    {
                        std::cout << response.class_results[i].class_type[kk].data <<
                                     " [" << response.class_results[i].confidence[kk] << "]" << std::endl;
                    }
                }

                pcl::PointCloud<PointT>::Ptr processed(new pcl::PointCloud<PointT>);
                pcl::transformPointCloud(*frame, *processed, t);

                std::stringstream name;
                name << "cloud_" << frames_.size();
                vis_->addPointCloud(processed, name.str(), v1_);

                vis_->removeAllPointClouds(v2_);
                vis_->removeAllShapes(v2_);

                for(size_t i=0; i < cluster_indices_ros_.size(); i++)
                {
                    pcl::PointCloud<PointT>::Ptr processed(new pcl::PointCloud<PointT>);
                    pcl::PointIndices cluster_indices;
                    cluster_indices.indices = cluster_indices_ros_[i].data;
                    pcl::copyPointCloud(*frame, cluster_indices, *processed);
                    pcl::transformPointCloud(*processed, *processed, t);

                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pClassifiedPCl (new pcl::PointCloud<pcl::PointXYZRGB>(*processed));
                    float r = std::rand() % 255;
                    float g = std::rand() % 255;
                    float b = std::rand() % 255;
                    for(size_t kk=0; kk < pClassifiedPCl->points.size(); kk++)
                    {
                        pClassifiedPCl->points[kk].r = r;
                        pClassifiedPCl->points[kk].g = g;
                        pClassifiedPCl->points[kk].b = b;
                    }

                    std::stringstream name;
                    name << "cluster_" << i;
                    pcl::visualization::PointCloudColorHandlerRGBField<PointT> rand_handler(pClassifiedPCl);
                    vis_->addPointCloud(pClassifiedPCl, rand_handler, name.str(), v2_);

                    for (size_t kk = 0; kk < class_results_ros_[i].class_type.size(); kk++)
                    {
                        std::stringstream prob_str;
                        prob_str.precision (2);
                        prob_str << class_results_ros_[i].class_type[kk].data << " [" << class_results_ros_[i].confidence[kk] << "]";

                        std::stringstream cluster_text;
                        cluster_text << "cluster_" << i << "_" << kk << "_text";
                        vis_->addText(prob_str.str(), 10+100*i, 10+kk*25,
                                      10, r/255.f, g/255.f, b/255.f, cluster_text.str(), v2_);

                    }
                }

                vis_->removeAllPointClouds(v3_);
                vis_->removeAllShapes(v3_);
                vis_->addPointCloud<OctreePointT>(cloud_octree, "octre_cloud", v3_);

                for(size_t i=0; i < cluster_indices.size(); i++)
                {
                    pcl::PointCloud<PointT>::Ptr processed(new pcl::PointCloud<PointT>);
                    pcl::PointIndices cluster_indices_intern;
                    cluster_indices_intern = cluster_indices[i];
                    pcl::copyPointCloud(*cloud_octree, cluster_indices_intern, *processed);
                    //pcl::transformPointCloud(*processed, *processed, t);

                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pClassifiedPCl (new pcl::PointCloud<pcl::PointXYZRGB>(*processed));
                    float r = std::rand() % 255;
                    float g = std::rand() % 255;
                    float b = std::rand() % 255;
                    for(size_t kk=0; kk < pClassifiedPCl->points.size(); kk++)
                    {
                        pClassifiedPCl->points[kk].r = r;
                        pClassifiedPCl->points[kk].g = g;
                        pClassifiedPCl->points[kk].b = b;
                    }

                    std::stringstream name;
                    name << "cluster_octree_" << i;
                    pcl::visualization::PointCloudColorHandlerRGBField<PointT> rand_handler(pClassifiedPCl);
                    vis_->addPointCloud(pClassifiedPCl, rand_handler, name.str(), v3_);

                    for (size_t kk = 0; kk < response.class_results[i].class_type.size(); kk++)
                    {
                        std::stringstream prob_str;
                        prob_str.precision (2);
                        prob_str << response.class_results[i].class_type[kk].data << " [" << response.class_results[i].confidence[kk] << "]";

                        std::stringstream cluster_text;
                        cluster_text << "cluster_octree" << i << "_" << kk << "_text";
                        vis_->addText(prob_str.str(), 10+100*i, 10+kk*25,
                                      10, r/255.f, g/255.f, b/255.f, cluster_text.str(), v3_);

                    }
                }

                vis_->spin();
            }
        }
        else
        {
            //TODO: just return the results
            response.class_results = class_results_ros_;
            response.centroid = srv.response.centroid;

            if(visualize_output_)
            {
                vis_->removeAllPointClouds(v2_);
                vis_->removeAllShapes();

                for(size_t i=0; i < cluster_indices_ros_.size(); i++)
                {
                    pcl::PointCloud<PointT>::Ptr processed(new pcl::PointCloud<PointT>);
                    pcl::PointIndices cluster_indices;
                    cluster_indices.indices = cluster_indices_ros_[i].data;
                    pcl::copyPointCloud(*frame, cluster_indices, *processed);
                    pcl::transformPointCloud(*processed, *processed, t);

                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pClassifiedPCl (new pcl::PointCloud<pcl::PointXYZRGB>(*processed));
                    float r = std::rand() % 255;
                    float g = std::rand() % 255;
                    float b = std::rand() % 255;
                    for(size_t kk=0; kk < pClassifiedPCl->points.size(); kk++)
                    {
                        pClassifiedPCl->points[kk].r = r;
                        pClassifiedPCl->points[kk].g = g;
                        pClassifiedPCl->points[kk].b = b;
                    }

                    std::cout << "Cluster " << i << ": " << std::endl;

                    std::stringstream name;
                    name << "cluster_" << i;
                    pcl::visualization::PointCloudColorHandlerRGBField<PointT> rand_handler(pClassifiedPCl);
                    vis_->addPointCloud(pClassifiedPCl, rand_handler, name.str(), v2_);

                    std::cout << "size results:" << class_results_ros_[i].class_type.size() << std::endl;
                    for (size_t kk = 0; kk < class_results_ros_[i].class_type.size(); kk++)
                    {
                        std::stringstream prob_str;
                        prob_str.precision (2);
                        prob_str << class_results_ros_[i].class_type[kk].data << " [" << class_results_ros_[i].confidence[kk] << "]";

                        std::stringstream cluster_text;
                        cluster_text << "cluster_" << i << "_" << kk << "_text";
                        vis_->addText(prob_str.str(), 10+100*i, 10+kk*25,
                                      10, r/255.f, g/255.f, b/255.f, cluster_text.str(), v2_);

                    }
                }

                vis_->spin();
            }
        }
        return true;
    }

  public:
    MVObjectClassifier()
    {
      //default values
      chop_at_z_ = 1.f;
      octree_resolution_ = 0.005f;
      visualize_output_ = false;
    }

    void initialize(int argc, char ** argv)
    {
        ros::init (argc, argv, "mv_object_classifier");
        n_ = new ros::NodeHandle ( "~" );
        n_->getParam ( "chop_z", chop_at_z_ );
        n_->getParam ( "visualize_output", visualize_output_ );

        add_cloud_service_ = n_->advertiseService("mv_add_cloud", &MVObjectClassifier::add_cloud, this);
        session_id_ = "";

        if(visualize_output_)
        {
            vis_.reset(new pcl::visualization::PCLVisualizer("accumulated"));
            vis_->createViewPort(0,0,0.33,1,v1_);
            vis_->createViewPort(0.33,0,0.66,1,v2_);
            vis_->createViewPort(0.66,0,1,1,v3_);
        }

        ros::spin();

    }
};

int
main (int argc, char ** argv)
{
  MVObjectClassifier m;
  m.initialize (argc, argv);

  return 0;
}
