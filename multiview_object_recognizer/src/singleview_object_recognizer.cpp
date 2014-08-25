#include "singleview_object_recognizer.h"

#define USE_SIFT_GPU
//#define SOC_VISUALIZE

bool USE_SEGMENTATION_ = false;

bool Recognizer::multiplaneSegmentation()
{
    //Multiplane segmentation
    faat_pcl::MultiPlaneSegmentation<PointT> mps;
    mps.setInputCloud(pInputCloud_);
    mps.setMinPlaneInliers(1000);
    mps.setResolution(go_resolution_);
    mps.setNormals(pSceneNormals_);
    mps.setMergePlanes(true);
    mps.segment();
    planes_found_ = mps.getModels();
}

void Recognizer::constructHypotheses()
{
    //    if(chop_at_z_ > 0)
    //    {
    //        pcl::PassThrough<PointT> pass_;
    //        pass_.setFilterLimits (0.f, static_cast<float>(chop_at_z_));
    //        pass_.setFilterFieldName ("z");
    //        pass_.setInputCloud (pInputCloud);
    //        pass_.setKeepOrganized (true);
    //        pass_.filter (*pInputCloud);
    //    }

        if(pSceneNormals_->points.size() == 0)
        {
            std::cout << "No normals point cloud for scene given. Calculate normals of scene..." << std::endl;
            pSceneNormals_.reset (new pcl::PointCloud<pcl::Normal>);
            pcl::NormalEstimationOMP<PointT, pcl::Normal> ne;
            ne.setRadiusSearch(0.02f);
            ne.setInputCloud (pInputCloud_);
            ne.compute (*pSceneNormals_);
        }

    //    if(USE_SEGMENTATION_)
    //    {
    //        std::vector<pcl::PointIndices> indices;
    //        Eigen::Vector4f table_plane;
    //        doSegmentation<PointT>(pInputCloud_, pSceneNormals_, indices, table_plane);

    //        std::vector<int> indices_above_plane;
    //        for (int k = 0; k < pInputCloud_->points.size (); k++)
    //        {
    //            Eigen::Vector3f xyz_p = pInputCloud_->points[k].getVector3fMap ();
    //            if (!pcl_isfinite (xyz_p[0]) || !pcl_isfinite (xyz_p[1]) || !pcl_isfinite (xyz_p[2]))
    //                continue;

    //            float val = xyz_p[0] * table_plane[0] + xyz_p[1] * table_plane[1] + xyz_p[2] * table_plane[2] + table_plane[3];
    //            if (val >= 0.01)
    //                indices_above_plane.push_back (static_cast<int> (k));
    //        }

    //        multi_recog_->setSegmentation(indices);
    //        multi_recog_->setIndices(indices_above_plane);

    //    }

        multi_recog_->setSceneNormals(pSceneNormals_);
        multi_recog_->setInputCloud (pInputCloud_);
        multi_recog_->setSaveHypotheses(true);
        std::cout << "Is organized: " << pInputCloud_->isOrganized();
        {
            pcl::ScopeTime ttt ("Recognition");
            multi_recog_->recognize ();
        }
        multi_recog_->getSavedHypotheses(hypotheses_);
        multi_recog_->getKeypointCloud(pKeypointsMultipipe_);
        multi_recog_->getKeypointIndices(keypointIndices_);

        models_ = multi_recog_->getModels ();
        transforms_ = multi_recog_->getTransforms ();
        std::cout << "Number of recognition hypotheses " << models_->size() << std::endl;

//        aligned_models_.resize (models_->size ());
        model_ids_.resize (models_->size ());
        for (size_t kk = 0; kk < models_->size (); kk++)
        {
//            ConstPointInTPtr model_cloud = models_->at (kk)->getAssembled (go_resolution_);
//            typename pcl::PointCloud<PointT>::Ptr model_aligned (new pcl::PointCloud<PointT>);
//            pcl::transformPointCloud (*model_cloud, *model_aligned, transforms_->at (kk));
//            aligned_models_[kk] = model_aligned;
            model_ids_[kk] = models_->at (kk)->id_;
        }
}

bool Recognizer::hypothesesVerification(std::vector<bool> &mask_hv)
{
    typename pcl::PointCloud<PointT>::Ptr occlusion_cloud (new pcl::PointCloud<PointT>(*pInputCloud_));

    //initialize go
    boost::shared_ptr<faat_pcl::GlobalHypothesesVerification_1<PointT, PointT> > go (
                    new faat_pcl::GlobalHypothesesVerification_1<PointT,
                    PointT>);

    go->setSmoothSegParameters(0.1, 0.035, 0.005);
    //go->setRadiusNormals(0.03f);
    go->setResolution (go_resolution_);
    go->setInlierThreshold (0.015);
    go->setRadiusClutter (0.03f);
    go->setRegularizer (3);
    go->setClutterRegularizer (5);
    go->setDetectClutter (true);
    go->setOcclusionThreshold (0.01f);
    go->setOptimizerType(0);
    go->setUseReplaceMoves(true);
    go->setRadiusNormals(0.02);
    go->setRequiresNormals(false);
    go->setInitialStatus(false);
    go->setIgnoreColor(true);
    go->setColorSigma(0.5f, 0.5f);
    go->setHistogramSpecification(true);
    go->setVisualizeGoCues(0);
    go->setUseSuperVoxels(false);
    go->setSceneCloud (pInputCloud_);
    go->setNormalsForClutterTerm(pSceneNormals_);
    go->setOcclusionCloud (occlusion_cloud);
    //addModels
    go->addModels (aligned_models_, true);

    //append planar models
    if(add_planes_)
    {
        multiplaneSegmentation();
        go->addPlanarModels(planes_found_);
        for(size_t kk=0; kk < planes_found_.size(); kk++)
        {
            std::stringstream plane_id;
            plane_id << "plane_" << kk;
            model_ids_.push_back(plane_id.str());
        }
    }

    if(model_ids_.size() == 0)
    {
        std::cout << "No models to verify, returning... " << std::endl;
        std::cout << "Cancelling service request." << std::endl;
        return true;
    }

    go->setObjectIds(model_ids_);
    //verify
    {
        pcl::ScopeTime t("Go verify");
        go->verify ();
    }
    go->getMask (mask_hv);

    std::vector<int> coming_from;
    coming_from.resize(aligned_models_.size() + planes_found_.size());
    for(size_t j=0; j < aligned_models_.size(); j++)
        coming_from[j] = 0;

    for(size_t j=0; j < planes_found_.size(); j++)
        coming_from[aligned_models_.size() + j] = 1;

    for (size_t j = 0; j < mask_hv.size (); j++)
    {
        if(!mask_hv[j])
            continue;

        if(coming_from[j] == 1) // it is a plane - therefore discard
        {
            mask_hv[j]=0;
        }
    }
    return true;
}

bool Recognizer::recognize ()
{

    std::vector<bool> mask_hv;

    constructHypotheses();

    std::vector<std::string> full_model_path (model_ids_.size());
    for(size_t model_id=0; model_id < model_ids_.size(); model_id++)
    {
        std::stringstream model_name;
        model_name << models_dir_ << model_ids_[model_id];
        full_model_path[model_id] = model_name.str();
    }

    setModelsAndTransforms(full_model_path, *transforms_);
    hypothesesVerification(mask_hv);

    //boost::shared_ptr<std::vector<ModelTPtr> > verified_models(new std::vector<ModelTPtr>);

  //  if(model_ids_)
    {
        for (size_t j = 0; j < mask_hv.size (); j++)
        {
            if(mask_hv[j])
            {
                //verified_models->push_back(models_->at(j));
                model_ids_verified_.push_back(model_ids_[j]);
                transforms_verified_.push_back(transforms_->at(j));
            }
        }
    }

    std::cout << "Number of models:" << model_ids_.size() <<
                 "Number of verified models:" << model_ids_verified_.size() << std::endl;

    //parse verified_models and generate response to service call
      //vector of id + pose

//    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pRecognizedModels (new pcl::PointCloud<pcl::PointXYZRGB>);

//    for (size_t j = 0; j < verified_models->size (); j++)
//    {
//      ConstPointInTPtr model_cloud = verified_models->at(j)->getAssembled (0.01);
//      typename pcl::PointCloud<PointT>::Ptr model_aligned (new pcl::PointCloud<PointT>);
//      pcl::transformPointCloud (*model_cloud, *model_aligned, transform_verified_[j]);
//      *pRecognizedModels += *model_aligned;
//    }
//    sensor_msgs::PointCloud2 recognizedModelsRos;
//    pcl::toROSMsg (*pRecognizedModels, recognizedModelsRos);
//    recognizedModelsRos.header.frame_id = "camera_link";
//    vis_pc_pub_.publish(recognizedModelsRos);

    visualizeHypotheses();
  }



  void Recognizer::initialize ()
  {
    boost::function<bool (const Eigen::Vector3f &)> campos_constraints;
    campos_constraints = camPosConstraints ();

    multi_recog_.reset (new faat_pcl::rec_3d_framework::MultiRecognitionPipeline<PointT>);
    boost::shared_ptr < pcl::CorrespondenceGrouping<PointT, PointT> > cast_cg_alg;
    boost::shared_ptr < faat_pcl::GraphGeometricConsistencyGrouping<PointT, PointT> > gcg_alg (
                                                                                               new faat_pcl::GraphGeometricConsistencyGrouping<
                                                                                                   PointT, PointT>);

    gcg_alg->setGCThreshold (cg_size_);
    gcg_alg->setGCSize (0.015);
    gcg_alg->setRansacThreshold (0.015);
    gcg_alg->setUseGraph (true);
    gcg_alg->setDistForClusterFactor (0);
    gcg_alg->setMaxTaken(2);
    gcg_alg->setMaxTimeForCliquesComputation(100);
    gcg_alg->setDotDistance (0.2);
    cast_cg_alg = boost::static_pointer_cast<pcl::CorrespondenceGrouping<PointT, PointT> > (gcg_alg);

    if (do_sift_)
    {

      std::string idx_flann_fn = "sift_flann.idx";
      std::string desc_name = "sift";

      boost::shared_ptr < faat_pcl::rec_3d_framework::RegisteredViewsSource<pcl::PointXYZRGBNormal, PointT, PointT>
          > mesh_source (new faat_pcl::rec_3d_framework::RegisteredViewsSource<pcl::PointXYZRGBNormal, pcl::PointXYZRGB, pcl::PointXYZRGB>);
      mesh_source->setPath (models_dir_);
      mesh_source->setModelStructureDir (sift_structure_);
      mesh_source->setLoadViews (false);
      mesh_source->generate (training_dir_sift_);

      boost::shared_ptr < faat_pcl::rec_3d_framework::Source<PointT> > cast_source;
      cast_source = boost::static_pointer_cast<faat_pcl::rec_3d_framework::RegisteredViewsSource<pcl::PointXYZRGBNormal, PointT, PointT> > (mesh_source);

#ifdef USE_SIFT_GPU

      if(!sift_) //--create a new SIFT-GPU context
      {
          static char kw[][16] = {"-m", "-fo", "-1", "-s", "-v", "1", "-pack"};
          char * argvv[] = {kw[0], kw[1], kw[2], kw[3],kw[4],kw[5],kw[6], NULL};

          int argcc = sizeof(argvv) / sizeof(char*);
          sift_ = new SiftGPU ();
          sift_->ParseParam (argcc, argvv);

          //create an OpenGL context for computation
          if (sift_->CreateContextGL () != SiftGPU::SIFTGPU_FULL_SUPPORTED)
            throw std::runtime_error ("PSiftGPU::PSiftGPU: No GL support!");
      }

      boost::shared_ptr < faat_pcl::rec_3d_framework::SIFTLocalEstimation<PointT, pcl::Histogram<128> > > estimator;
      estimator.reset (new faat_pcl::rec_3d_framework::SIFTLocalEstimation<PointT, pcl::Histogram<128> >(sift_));

      boost::shared_ptr < faat_pcl::rec_3d_framework::LocalEstimator<PointT, pcl::Histogram<128> > > cast_estimator;
      cast_estimator = boost::dynamic_pointer_cast<faat_pcl::rec_3d_framework::SIFTLocalEstimation<PointT, pcl::Histogram<128> > > (estimator);
#else
      boost::shared_ptr < faat_pcl::rec_3d_framework::OpenCVSIFTLocalEstimation<PointT, pcl::Histogram<128> > > estimator;
      estimator.reset (new faat_pcl::rec_3d_framework::OpenCVSIFTLocalEstimation<PointT, pcl::Histogram<128> >);

      boost::shared_ptr < faat_pcl::rec_3d_framework::LocalEstimator<PointT, pcl::Histogram<128> > > cast_estimator;
      cast_estimator = boost::dynamic_pointer_cast<faat_pcl::rec_3d_framework::OpenCVSIFTLocalEstimation<PointT, pcl::Histogram<128> > > (estimator);
#endif

      boost::shared_ptr<faat_pcl::rec_3d_framework::LocalRecognitionPipeline<flann::L1, PointT, pcl::Histogram<128> > > new_sift_local_;
      new_sift_local_.reset (new faat_pcl::rec_3d_framework::LocalRecognitionPipeline<flann::L1, PointT, pcl::Histogram<128> > (idx_flann_fn));
      new_sift_local_->setDataSource (cast_source);
      new_sift_local_->setTrainingDir (training_dir_sift_);
      new_sift_local_->setDescriptorName (desc_name);
      new_sift_local_->setICPIterations (0);
      new_sift_local_->setFeatureEstimator (cast_estimator);
      new_sift_local_->setUseCache (true);
      new_sift_local_->setCGAlgorithm (cast_cg_alg);
      new_sift_local_->setKnn (5);
      new_sift_local_->setUseCache (true);
      new_sift_local_->initialize (false);

      boost::shared_ptr < faat_pcl::rec_3d_framework::Recognizer<PointT> > cast_recog;
      cast_recog = boost::static_pointer_cast<faat_pcl::rec_3d_framework::LocalRecognitionPipeline<flann::L1, PointT, pcl::Histogram<128> > > (
                                                                                                                                        new_sift_local_);
      std::cout << "Feature Type: " << cast_recog->getFeatureType() << std::endl;
      multi_recog_->addRecognizer (cast_recog);
    }

    if(do_ourcvfh_ && USE_SEGMENTATION_)
    {
      boost::shared_ptr<faat_pcl::rec_3d_framework::PartialPCDSource<pcl::PointXYZRGBNormal, pcl::PointXYZRGB> >
                          source (
                              new faat_pcl::rec_3d_framework::PartialPCDSource<
                              pcl::PointXYZRGBNormal,
                              pcl::PointXYZRGB>);
      source->setPath (models_dir_);
      source->setModelScale (1.f);
      source->setRadiusSphere (1.f);
      source->setTesselationLevel (1);
      source->setDotNormal (-1.f);
      source->setUseVertices(false);
      source->setLoadViews (false);
      source->setCamPosConstraints (campos_constraints);
      source->setLoadIntoMemory(false);
      source->setGenOrganized(true);
      source->setWindowSizeAndFocalLength(640, 480, 575.f);
      source->generate (training_dir_ourcvfh_);

      boost::shared_ptr<faat_pcl::rec_3d_framework::Source<pcl::PointXYZRGB> > cast_source;
      cast_source = boost::static_pointer_cast<faat_pcl::rec_3d_framework::PartialPCDSource<pcl::PointXYZRGBNormal, pcl::PointXYZRGB> > (source);

      //configure normal estimator
      boost::shared_ptr<faat_pcl::rec_3d_framework::PreProcessorAndNormalEstimator<PointT, pcl::Normal> > normal_estimator;
      normal_estimator.reset (new faat_pcl::rec_3d_framework::PreProcessorAndNormalEstimator<PointT, pcl::Normal>);
      normal_estimator->setCMR (false);
      normal_estimator->setDoVoxelGrid (false);
      normal_estimator->setRemoveOutliers (false);
      normal_estimator->setValuesForCMRFalse (0.001f, 0.02f);
      normal_estimator->setForceUnorganized(true);

      //boost::shared_ptr<faat_pcl::rec_3d_framework::ColorOURCVFHEstimator<PointT, pcl::Histogram<1327> > > vfh_estimator;
      //vfh_estimator.reset (new faat_pcl::rec_3d_framework::ColorOURCVFHEstimator<PointT, pcl::Histogram<1327> >);

      boost::shared_ptr<faat_pcl::rec_3d_framework::OrganizedColorOURCVFHEstimator<PointT, pcl::Histogram<1327> > > vfh_estimator;
      vfh_estimator.reset (new faat_pcl::rec_3d_framework::OrganizedColorOURCVFHEstimator<PointT, pcl::Histogram<1327> >);
      vfh_estimator->setNormalEstimator (normal_estimator);
      vfh_estimator->setNormalizeBins(true);
      vfh_estimator->setUseRFForColor (true);
      //vfh_estimator->setRefineClustersParam (2.5f);
      vfh_estimator->setRefineClustersParam (100.f);
      vfh_estimator->setAdaptativeMLS (false);

      vfh_estimator->setAxisRatio (1.f);
      vfh_estimator->setMinAxisValue (1.f);

      {
          //segmentation parameters for training
          std::vector<float> eps_thresholds, cur_thresholds, clus_thresholds;
          eps_thresholds.push_back (0.15);
          cur_thresholds.push_back (0.015f);
          cur_thresholds.push_back (1.f);
          clus_thresholds.push_back (10.f);

          vfh_estimator->setClusterToleranceVector (clus_thresholds);
          vfh_estimator->setEpsAngleThresholdVector (eps_thresholds);
          vfh_estimator->setCurvatureThresholdVector (cur_thresholds);
      }

      std::string desc_name = "rf_our_cvfh_color_normalized";

      boost::shared_ptr<faat_pcl::rec_3d_framework::OURCVFHEstimator<pcl::PointXYZRGB, pcl::Histogram<1327> > > cast_estimator;
      cast_estimator = boost::dynamic_pointer_cast<faat_pcl::rec_3d_framework::OrganizedColorOURCVFHEstimator<pcl::PointXYZRGB, pcl::Histogram<1327> > > (vfh_estimator);

      boost::shared_ptr<faat_pcl::rec_3d_framework::GlobalNNCVFHRecognizer<faat_pcl::Metrics::HistIntersectionUnionDistance, PointT, pcl::Histogram<1327> > > rf_color_ourcvfh_global_;
      rf_color_ourcvfh_global_.reset(new faat_pcl::rec_3d_framework::GlobalNNCVFHRecognizer<faat_pcl::Metrics::HistIntersectionUnionDistance, PointT, pcl::Histogram<1327> >);
      rf_color_ourcvfh_global_->setDataSource (cast_source);
      rf_color_ourcvfh_global_->setTrainingDir (training_dir_ourcvfh_);
      rf_color_ourcvfh_global_->setDescriptorName (desc_name);
      rf_color_ourcvfh_global_->setFeatureEstimator (cast_estimator);
      rf_color_ourcvfh_global_->setNN (50);
      rf_color_ourcvfh_global_->setICPIterations (0);
      rf_color_ourcvfh_global_->setNoise (0.0f);
      rf_color_ourcvfh_global_->setUseCache (true);
      rf_color_ourcvfh_global_->setMaxHyp(15);
      rf_color_ourcvfh_global_->setMaxDescDistance(0.75f);
      rf_color_ourcvfh_global_->initialize (false);
      rf_color_ourcvfh_global_->setDebugLevel(2);
      {
          //segmentation parameters for recognition
          std::vector<float> eps_thresholds, cur_thresholds, clus_thresholds;
          eps_thresholds.push_back (0.15);
          cur_thresholds.push_back (0.015f);
          cur_thresholds.push_back (0.02f);
          cur_thresholds.push_back (1.f);
          clus_thresholds.push_back (10.f);

          vfh_estimator->setClusterToleranceVector (clus_thresholds);
          vfh_estimator->setEpsAngleThresholdVector (eps_thresholds);
          vfh_estimator->setCurvatureThresholdVector (cur_thresholds);

          vfh_estimator->setAxisRatio (0.8f);
          vfh_estimator->setMinAxisValue (0.8f);

          vfh_estimator->setAdaptativeMLS (false);
      }

      boost::shared_ptr < faat_pcl::rec_3d_framework::Recognizer<PointT> > cast_recog;
      cast_recog = boost::static_pointer_cast<faat_pcl::rec_3d_framework::GlobalNNCVFHRecognizer<faat_pcl::Metrics::HistIntersectionUnionDistance, PointT, pcl::Histogram<1327> > > (rf_color_ourcvfh_global_);
      multi_recog_->addRecognizer(cast_recog);
    }

    multi_recog_->setCGAlgorithm(gcg_alg);
    multi_recog_->setVoxelSizeICP(0.005f);
    multi_recog_->setICPType(1);
    multi_recog_->setICPIterations(icp_iterations_);
    multi_recog_->initialize();

//    recognize_  = n_->advertiseService ("mp_recognition", &Recognizer::recognize, this);
//    vis_pc_pub_ = n_->advertise<sensor_msgs::PointCloud2>( "sv_recogniced_object_instances_", 1 );
//    std::cout << "Ready to get service calls..." << std::endl;
//    ros::spin ();
  }


  void Recognizer::visualizeHypotheses()
  {
#ifdef SOC_VISUALIZE
    vis_->removeAllPointClouds();
    vis_->addPointCloud(scene, "scene", v1_);

    for(size_t kk=0; kk < planes_found.size(); kk++)
    {
        std::stringstream pname;
        pname << "plane_" << kk;
        pcl::visualization::PointCloudColorHandlerRandom<PointT> scene_handler(planes_found[kk].plane_cloud_);
        vis_->addPointCloud<PointT> (planes_found[kk].plane_cloud_, scene_handler, pname.str(), v2_);
        pname << "chull";
        vis_->addPolygonMesh (*planes_found[kk].convex_hull_, pname.str(), v2_);
    }

    if(models)
    {
        for (size_t j = 0; j < mask_hv.size (); j++)
        {
            std::stringstream name;
            name << "cloud_" << j;

            if(!mask_hv[j])
            {
                if(coming_from[j] == 0)
                {
                    ConstPointInTPtr model_cloud = models->at (j)->getAssembled (assembled_resolution);
                    typename pcl::PointCloud<PointT>::Ptr model_aligned (new pcl::PointCloud<PointT>);
                    pcl::transformPointCloud (*model_cloud, *model_aligned, transforms->at (j));
                    pcl::visualization::PointCloudColorHandlerRGBField<PointT> random_handler (model_aligned);
                    vis_->addPointCloud<PointT> (model_aligned, random_handler, name.str (), v2_);
                }
                continue;
            }

            if(coming_from[j] == 0)
            {
                verified_models->push_back(models->at(j));
                verified_transforms->push_back(transforms->at(j));

                ConstPointInTPtr model_cloud = models->at (j)->getAssembled (-1);
                typename pcl::PointCloud<PointT>::Ptr model_aligned (new pcl::PointCloud<PointT>);
                pcl::transformPointCloud (*model_cloud, *model_aligned, transforms->at (j));
                std::cout << models->at (j)->id_ << std::endl;

                pcl::visualization::PointCloudColorHandlerRGBField<PointT> random_handler (model_aligned);
                vis_->addPointCloud<PointT> (model_aligned, random_handler, name.str (), v3_);
            }
            else
            {
                std::stringstream pname;
                pname << "plane_v2_" << j;
                pcl::visualization::PointCloudColorHandlerRandom<PointT> scene_handler(planes_found[j - models->size()].plane_cloud_);
                vis_->addPointCloud<PointT> (planes_found[j - models->size()].plane_cloud_, scene_handler, pname.str(), v3_);
                pname << "chull_v2";
                vis_->addPolygonMesh (*planes_found[j - models->size()].convex_hull_, pname.str(), v3_);
            }
        }
    }
    vis_->spin ();
#endif

  }
