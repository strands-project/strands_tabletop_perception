#include "recognizer_ros.h"
#include "pcl_conversions.h"
#include <cv_bridge/cv_bridge.h>


void RecognizerROS::createDebugImageAndPublish(recognition_srv_definitions::recognize::Response & response,
                                pcl::PointCloud<pcl::PointXYZRGB>::Ptr & scene_)
{
    std::vector<std::string> model_ids_;
    std::vector<Eigen::Matrix4f> transforms_;
    std::vector<float> confidences_;
    std::vector<std::pair<cv::Point, cv::Point> > box_rectangles_;

    bool extended_request_ = true;

    for(size_t i=0; i < response.ids.size(); i++)
    {
        model_ids_.push_back(response.ids[i].data);

        Eigen::Quaternionf q(response.transforms[i].rotation.w,
                             response.transforms[i].rotation.x,
                             response.transforms[i].rotation.y,
                             response.transforms[i].rotation.z);

        Eigen::Vector3f translation(response.transforms[i].translation.x,
                                    response.transforms[i].translation.y,
                                    response.transforms[i].translation.z);


        Eigen::Matrix4f trans;
        trans.block<3,3>(0,0) = q.toRotationMatrix();
        trans.block<3,1>(0,3) = translation;
        transforms_.push_back(trans);

        if(extended_request_)
        {
            confidences_.push_back(response.confidence[i]);
            pcl::PointCloud<pcl::PointXYZRGB> model;
            pcl::fromROSMsg(response.models_cloud[i], model);

            int cx_, cy_;
            cx_ = 640;
            cy_ = 480;
            float focal_length_ = 525.f;

            float cx, cy;
            cx = static_cast<float> (cx_) / 2.f; //- 0.5f;
            cy = static_cast<float> (cy_) / 2.f; // - 0.5f;

            int min_u, min_v, max_u, max_v;
            min_u = min_v = cx_;
            max_u = max_v = 0;

            for(size_t j=0; j < model.points.size(); j++)
            {

                float x = model.points[j].x;
                float y = model.points[j].y;
                float z = model.points[j].z;
                int u = static_cast<int> (focal_length_ * x / z + cx);
                int v = static_cast<int> (focal_length_ * y / z + cy);

                if (u >= cx_ || v >= cy_ || u < 0 || v < 0)
                  continue;

                if(u < min_u)
                    min_u = u;

                if(v < min_v)
                    min_v = v;

                if(u > max_u)
                    max_u = u;

                if(v > max_v)
                    max_v = v;
            }

            cv::Point min, max;
            min.x = min_u;
            min.y = min_v;

            max.x = max_u;
            max.y = max_v;

            std::cout << min_u << " " << min_v << " .... " << max_u << " " << max_v << std::endl;

            box_rectangles_.push_back(std::make_pair(min,max));
        }
    }

    cv::Mat_<cv::Vec3b> image;
    PCLOpenCV::ConvertPCLCloud2Image<pcl::PointXYZRGB>(scene_, image);

    for(size_t kk=0; kk < transforms_.size(); kk++)
    {


        cv::Point text_start;
        text_start.x = box_rectangles_[kk].first.x;
        text_start.y = std::max(0, box_rectangles_[kk].first.y - 10);
        std::string model_id(model_ids_[kk], 0, 15);
        cv::putText(image, model_id, text_start,
            cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(255,0,255), 1, CV_AA);

        cv::rectangle(image, box_rectangles_[kk].first, box_rectangles_[kk].second, cv::Scalar( 0, 255, 255 ), 2);
    }

    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
    image_pub_.publish(msg);
}


bool
RecognizerROS::retrainROS (recognition_srv_definitions::retrain_recognizer::Request & req,
         recognition_srv_definitions::retrain_recognizer::Response & response)
{
      std::vector<std::string> model_ids;
      std::cout << "Number of ids:" << req.load_ids.size() << std::endl;

      for(size_t i=0; i < req.load_ids.size(); i++)
      {
          model_ids.push_back(req.load_ids[i].data);
          std::cout << req.load_ids[i].data << std::endl;
      }
      retrain(model_ids);
}

bool RecognizerROS::recognizeROS(recognition_srv_definitions::recognize::Request &req,
                              recognition_srv_definitions::recognize::Response &response)
{
    pcl::PointCloud<PointT>::Ptr scene (new pcl::PointCloud<PointT>);
    pcl::fromROSMsg (req.cloud, *scene);
    setInputCloud(scene);

    recognize();

    std::vector<std::string> verified_models;
    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > verified_transforms;
    getModelsAndTransforms( verified_models, verified_transforms);

//    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pRecognizedModels (new pcl::PointCloud<pcl::PointXYZRGB>);

    for (size_t j = 0; j < verified_models.size (); j++)
    {
      std_msgs::String ss;
      ss.data = verified_models[j];
      response.ids.push_back(ss);

      Eigen::Matrix4f trans = verified_transforms[j];
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
      response.transforms.push_back(tt);


//      ConstPointInTPtr model_cloud = verified_models[->at(j)]j]->getAssembled (0.01);
//      typename pcl::PointCloud<PointT>::Ptr model_aligned (new pcl::PointCloud<PointT>);
//      pcl::transformPointCloud (*model_cloud, *model_aligned, verified_transforms->at(j));
//      *pRecognizedModels += *model_aligned;
    }
//    sensor_msgs::PointCloud2 recognizedModelsRos;
//    pcl::toROSMsg (*pRecognizedModels, recognizedModelsRos);
//    recognizedModelsRos.header.frame_id = "camera_rgb_optical_frame";
//    vis_pc_pub_.publish(recognizedModelsRos);
}

void
RecognizerROS::initialize (int argc, char ** argv)
{
    n_.reset( new ros::NodeHandle ( "~" ) );
    n_->getParam ( "models_dir", models_dir_);
    n_->getParam ( "training_dir_sift", training_dir_sift_);
    n_->getParam ( "training_dir_shot", training_dir_shot_);
    n_->getParam ( "recognizer_structure_sift", sift_structure_);
    n_->getParam ( "training_dir_ourcvfh", training_dir_ourcvfh_);
    n_->getParam ( "chop_z", chop_at_z_ );
    n_->getParam ( "icp_iterations", icp_iterations_);
    n_->getParam ( "do_sift", do_sift_);
    n_->getParam ( "do_shot", do_shot_);
    n_->getParam ( "do_ourcvfh", do_ourcvfh_);
    n_->getParam ( "knn_sift", knn_sift_);
    n_->getParam ( "publish_debug", debug_publish_);

    n_->getParam ( "cg_size_thresh", cg_params_.cg_size_threshold_);
    n_->getParam ( "cg_size", cg_params_.cg_size_);
    n_->getParam ( "cg_ransac_threshold", cg_params_.ransac_threshold_);
    n_->getParam ( "cg_dist_for_clutter_factor", cg_params_.dist_for_clutter_factor_);
    n_->getParam ( "cg_max_taken", cg_params_.max_taken_);
    n_->getParam ( "cg_max_time_for_cliques_computation", cg_params_.max_time_for_cliques_computation_);
    n_->getParam ( "cg_dot_distance", cg_params_.dot_distance_);
    n_->getParam ( "use_cg_graph", cg_params_.use_cg_graph_);

    n_->getParam ( "hv_resolution", hv_params_.resolution_);
    n_->getParam ( "hv_inlier_threshold", hv_params_.inlier_threshold_);
    n_->getParam ( "hv_radius_clutter", hv_params_.radius_clutter_);
    n_->getParam ( "hv_regularizer", hv_params_.regularizer_);
    n_->getParam ( "hv_clutter_regularizer", hv_params_.clutter_regularizer_);
    n_->getParam ( "hv_occlusion_threshold", hv_params_.occlusion_threshold_);
    n_->getParam ( "hv_optimizer_type", hv_params_.optimizer_type_);
    n_->getParam ( "hv_color_sigma_l", hv_params_.color_sigma_l_);
    n_->getParam ( "hv_color_sigma_ab", hv_params_.color_sigma_ab_);
    n_->getParam ( "hv_use_supervoxels", hv_params_.use_supervoxels_);
    n_->getParam ( "hv_detect_clutter", hv_params_.detect_clutter_);
    n_->getParam ( "hv_ignore_color", hv_params_.ignore_color_);

    std::cout << chop_at_z_ << ", " << hv_params_.ignore_color_ << ", do_shot:" << do_shot_ << std::endl;
  if (models_dir_.compare ("") == 0)
  {
    PCL_ERROR ("Set -models_dir option in the command line, ABORTING");
    return;
  }

  if (do_sift_ && training_dir_sift_.compare ("") == 0)
  {
    PCL_ERROR ("do_sift is activated but training_dir_sift_ is empty! Set -training_dir_sift option in the command line, ABORTING");
    return;
  }

  if (do_ourcvfh_ && training_dir_ourcvfh_.compare ("") == 0)
  {
    PCL_ERROR ("do_ourcvfh is activated but training_dir_ourcvfh_ is empty! Set -training_dir_ourcvfh option in the command line, ABORTING");
    return;
  }

  if (do_shot_ && training_dir_shot_.compare ("") == 0)
  {
    PCL_ERROR ("do_shot is activated but training_dir_shot_ is empty! Set -training_dir_shot option in the command line, ABORTING");
    return;
  }
  vis_pc_pub_ = n_->advertise<sensor_msgs::PointCloud2>( "sv_recogniced_object_instances_", 1 );
  recognize_  = n_->advertiseService ("sv_recognition", &RecognizerROS::recognizeROS, this);

  initialize();
  std::cout << "Initialized single-view recognizer with these settings:" << std::endl
            << "==========================================================" << std::endl;
  printParams();
}

int
main (int argc, char ** argv)
{
  ros::init (argc, argv, "recognition_service");

  RecognizerROS m;
  m.initialize (argc, argv);
  ros::spin ();

  return 0;
}
