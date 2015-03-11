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
#include <v4r/utils/filesystem_utils.h>
#include <v4r/ORFramework/sift_local_estimator.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include "classifier_srv_definitions/mv_classify.h"

class MVObjectClassifierDemoFromFiles
{
private:
    typedef pcl::PointXYZRGB PointT;
	std::string files_folder_;
    ros::NodeHandle *n_;
    typedef pcl::Histogram<128> FeatureT;
    typedef flann::L1<float> DistT;
public:
    MVObjectClassifierDemoFromFiles()
    {
        files_folder_ = "";
    }

    bool initialize(int argc, char ** argv)
    {
        ros::init (argc, argv, "mv_obj_classifier_demo_from_files");
        n_ = new ros::NodeHandle ( "~" );
        n_->getParam ( "folder", files_folder_ );
        return true;
    }

    template<typename Type>
    void
    convertToFLANN (typename pcl::PointCloud<Type>::Ptr & cloud, flann::Matrix<float> &data)
    {
        data.rows = cloud->points.size ();
        data.cols = sizeof(cloud->points[0].histogram) / sizeof(float); // number of histogram bins

        std::cout << data.rows << " " << data.cols << std::endl;

        flann::Matrix<float> flann_data (new float[data.rows * data.cols], data.rows, data.cols);

        for (size_t i = 0; i < data.rows; ++i)
          for (size_t j = 0; j < data.cols; ++j)
          {
            flann_data.ptr ()[i * data.cols + j] = cloud->points[i].histogram[j];
          }

        data = flann_data;
    }

    void
    nearestKSearch (flann::Index<DistT> * index, float * descr, int descr_size, int k, flann::Matrix<int> &indices,
                               flann::Matrix<float> &distances)
    {
      flann::Matrix<float> p = flann::Matrix<float> (new float[descr_size], 1, descr_size);
      memcpy (&p.ptr ()[0], &descr[0], p.cols * p.rows * sizeof(float));

      index->knnSearch (p, indices, distances, k, flann::SearchParams (128));
      delete[] p.ptr ();
    }

    Eigen::Matrix4f pairwise( pcl::PointCloud<PointT>::Ptr & keyp_src,
                   pcl::PointCloud<PointT>::Ptr & keyp_tgt,
                   pcl::PointCloud<FeatureT>::Ptr & sigs_src,
                   pcl::PointCloud<FeatureT>::Ptr & sigs_tgt)
    {
        Eigen::Matrix4f transformation;
        flann::Matrix<float> flann_data;
        flann::Index<DistT> *flann_index;
        convertToFLANN< FeatureT > (sigs_src, flann_data);
        flann_index = new flann::Index<DistT> (flann_data, flann::KDTreeIndexParams (4));
        flann_index->buildIndex ();

        int K = 1;
        flann::Matrix<int> indices = flann::Matrix<int> (new int[K], 1, K);
        flann::Matrix<float> distances = flann::Matrix<float> (new float[K], 1, K);

        pcl::CorrespondencesPtr temp_correspondences (new pcl::Correspondences);
        for (size_t keypointId = 0; keypointId < keyp_tgt->points.size (); keypointId++)
        {
          FeatureT searchFeature = sigs_tgt->points[keypointId];
          int size_feat = sizeof(searchFeature.histogram) / sizeof(float);
          nearestKSearch (flann_index, searchFeature.histogram, size_feat, K, indices, distances);

          pcl::Correspondence corr;
          corr.distance = distances[0][0];
          corr.index_query = keypointId;
          corr.index_match = indices[0][0];
          temp_correspondences->push_back (corr);
        }

        pcl::registration::CorrespondenceRejectorSampleConsensus<PointT>::Ptr rej;
        rej.reset (new pcl::registration::CorrespondenceRejectorSampleConsensus<PointT> ());
        pcl::CorrespondencesPtr after_rej_correspondences (new pcl::Correspondences ());

        rej->setMaximumIterations (50000);
        rej->setInlierThreshold (0.02);
        rej->setInputTarget (keyp_src);
        rej->setInputSource (keyp_tgt);
        rej->setInputCorrespondences (temp_correspondences);
        rej->getCorrespondences (*after_rej_correspondences);

        transformation = rej->getBestTransformation ();
        pcl::registration::TransformationEstimationSVD<PointT, PointT> t_est;

        t_est.estimateRigidTransformation (*keyp_tgt, *keyp_src, *after_rej_correspondences, transformation);
        return transformation;
    }

    void gen_random(std::string & ss,  const int len) {
        static const char alphanum[] =
            "0123456789"
            "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
            "abcdefghijklmnopqrstuvwxyz";

        char *s = new char[len];
        for (int i = 0; i < len; ++i) {
            s[i] = alphanum[rand() % (sizeof(alphanum) - 1)];
        }

        s[len] = 0;
        ss = std::string(s);
        delete[] s;
    }

    geometry_msgs::Transform matrix4fToGeometryMsg(Eigen::Matrix4f & trans)
    {
        geometry_msgs::Transform tt;
        tt.translation.x = trans(0,3);
        tt.translation.y = trans(1,3);
        tt.translation.z = trans(2,3);

        Eigen::Matrix3f rotation = trans.block<3,3>(0,0);
        Eigen::Quaternionf q(rotation);
        tt.rotation.x = q.x();
        tt.rotation.y = q.y();
        tt.rotation.z = q.z();
        tt.rotation.w = q.w();
        return tt;
    }

    void run()
    {
        //iterate over files and register them, then call service incrementally
        std::string session_id;
        srand(time(NULL));
        gen_random(session_id, 10);

        std::vector<std::string> files;
        std::vector<pcl::PointCloud<FeatureT>::Ptr> signatures;
        std::vector<pcl::PointCloud<PointT>::Ptr> keypoints;
        std::vector<pcl::PointCloud<PointT>::Ptr> clouds;
        v4r::utils::getFilesInDirectory(files_folder_, files, "", "cloud.*.pcd", false);

        std::cout << files.size() << std::endl;
        clouds.resize(files.size());
        signatures.resize(files.size());
        keypoints.resize(files.size());
        for(size_t i=0; i < files.size(); i++)
        {
            std::stringstream file_str;
            file_str << files_folder_ << "/" << files[i];
            std::cout << file_str.str() << std::endl;
            pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
            pcl::PointCloud<PointT>::Ptr processed(new pcl::PointCloud<PointT>);
            pcl::io::loadPCDFile(file_str.str(), *cloud);

            pcl::PointCloud<PointT>::Ptr keyp(new pcl::PointCloud<PointT>);
            pcl::PointCloud<FeatureT>::Ptr sigs(new pcl::PointCloud<FeatureT>);

            boost::shared_ptr < faat_pcl::rec_3d_framework::SIFTLocalEstimation<PointT, FeatureT> > estimator;
            estimator.reset (new faat_pcl::rec_3d_framework::SIFTLocalEstimation<PointT, FeatureT>);
            estimator->estimate (cloud, processed, keyp, sigs);

            keypoints[i] = keyp;
            signatures[i] = sigs;
            clouds[i] = cloud;
        }

        //pairwise registration
        std::vector<Eigen::Matrix4f> poses_to_global;
        poses_to_global.resize(files.size(), Eigen::Matrix4f::Identity());

        for(size_t i=1; i < files.size(); i++)
        {
            Eigen::Matrix4f incr_pose = pairwise(keypoints[i-1],keypoints[i], signatures[i-1], signatures[i]);
            poses_to_global[i] = poses_to_global[i-1] * incr_pose;
        }

        /*pcl::visualization::PCLVisualizer vis("registered clouds");
        for(size_t i=0; i < files.size(); i++)
        {
            pcl::PointCloud<PointT>::Ptr processed(new pcl::PointCloud<PointT>);
            pcl::transformPointCloud(*clouds[i], *processed, poses_to_global[i]);
            std::stringstream name;
            name << "cloud_" << i;
            vis.addPointCloud(processed, name.str());
        }
        vis.spin();*/

        //with clouds and poses_to_global, call mv obj classifier incrementally
        ros::ServiceClient mv_obj_client = n_->serviceClient<classifier_srv_definitions::mv_classify>("/mv_object_classifier/mv_add_cloud");

        for(size_t i=0; i < files.size(); i++)
        {
            sensor_msgs::PointCloud2 cloud_ros;
            pcl::toROSMsg(*clouds[i], cloud_ros);

            classifier_srv_definitions::mv_classify add_cloud_srv;
            add_cloud_srv.request.cloud = cloud_ros;
            add_cloud_srv.request.session_id.data = session_id;

            geometry_msgs::Transform tt = matrix4fToGeometryMsg(poses_to_global[i]);
            add_cloud_srv.request.transform = tt;

            if(mv_obj_client.call(add_cloud_srv))
            {
                PCL_INFO("Call to mv obj client succeeded\n");
                for(size_t k=0; k < add_cloud_srv.response.centroid.size(); k++)
                {
                    std::cout << "Cluster #" << k << std::endl;
                    for (size_t kk = 0; kk < add_cloud_srv.response.class_results[k].class_type.size(); kk++)
                    {
                        std::cout << add_cloud_srv.response.class_results[k].class_type[kk].data <<
                                     " [" << add_cloud_srv.response.class_results[k].confidence[kk] << "]" << std::endl;
                    }
                }
            }
            else
            {
                PCL_ERROR("Could not call mv_object_classifier\n");
            }
        }
    }

};

int
main (int argc, char ** argv)
{
  MVObjectClassifierDemoFromFiles mv_demo;
  mv_demo.initialize(argc, argv);
  mv_demo.run();
  return 0;
}
