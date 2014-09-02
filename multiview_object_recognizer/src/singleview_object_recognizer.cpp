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
    //        doSegmentation<PointT>(pInputCloud, pSceneNormals_, indices, table_plane);

    //        std::vector<int> indices_above_plane;
    //        for (int k = 0; k < pInputCloud->points.size (); k++)
    //        {
    //            Eigen::Vector3f xyz_p = pInputCloud->points[k].getVector3fMap ();
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

    mask_hv.clear();
    //initialize go
    boost::shared_ptr<faat_pcl::GlobalHypothesesVerification_1<PointT, PointT> > go (
                    new faat_pcl::GlobalHypothesesVerification_1<PointT,
                    PointT>);

    assert(pSceneNormals_->points.size() == pInputCloud_->points.size());
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

//    pcl::visualization::PCLVisualizer::Ptr vis_temp (new pcl::visualization::PCLVisualizer);
//    int v1,v2;
//    vis_temp->createViewPort(0,0,0.5,1,v1);
//    vis_temp->createViewPort(0.5,0,1,1,v2);
//    for(size_t i=0; i<aligned_models_.size(); i++)
//    {
//        std::stringstream cloud_name;
//        cloud_name << i;
//        assert(aligned_models_[i]->points.size()>0);
//        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> handler_rgb_verified (aligned_models_[i]);
//        vis_temp->addPointCloud<pcl::PointXYZRGB> (aligned_models_[i], handler_rgb_verified, cloud_name.str(), v1);
//    }
//    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> handler_rgb (pInputCloud_);
//    vis_temp->addPointCloud<pcl::PointXYZRGB>(pInputCloud_, handler_rgb, "scene", v2);
//    vis_temp->addPointCloudNormals<pcl::PointXYZRGB,pcl::Normal>(pInputCloud_, pSceneNormals_, 30, 0.03, "scene_with_normals", v2);
//    vis_temp->spin();

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
    std::vector<bool> mask_hv_with_planes;
    go->getMask (mask_hv_with_planes);

    std::vector<int> coming_from;
    coming_from.resize(aligned_models_.size() + planes_found_.size());
    for(size_t j=0; j < aligned_models_.size(); j++)
        coming_from[j] = 0;

    for(size_t j=0; j < planes_found_.size(); j++)
        coming_from[aligned_models_.size() + j] = 1;

    for (size_t j = 0; j < mask_hv_with_planes.size (); j++)
    {
        if(coming_from[j] == 1) // it is a plane - therefore discard
        {
//            mask_hv[j]=0;
            if(j < aligned_models_.size())
            {
                std::cerr << "Model plane not at the end of hypotheses vector!! Check this part of code again!" << std::endl;
            }
            continue;
        }
        mask_hv.push_back(mask_hv_with_planes[j]);
    }
    return true;
}


bool Recognizer::hypothesesVerificationGpu(std::vector<bool> &mask_hv)
{
    typename pcl::PointCloud<PointT>::Ptr pOcclusionCloud (new pcl::PointCloud<PointT>(*pInputCloud_));
    typename pcl::PointCloud<PointT>::Ptr pInputCloud_ds (new pcl::PointCloud<PointT>);
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr pInputCloudWithNormals (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr pInputCloudWithNormals_ds (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl::PointCloud<pcl::Normal>::Ptr pInputNormals_ds (new pcl::PointCloud<pcl::Normal>);

    float res = 0.005f;

    assert(pInputCloud_->points.size() == pSceneNormals_->points.size());
    pInputCloudWithNormals->points.resize(pInputCloud_->points.size());
    size_t kept=0;
    for(size_t i=0; i<pInputCloud_->points.size(); i++)
    {
        if(pcl::isFinite(pInputCloud_->points[i])  && pcl::isFinite(pSceneNormals_->points[i]))
        {
            pInputCloudWithNormals->points[kept].getVector3fMap() = pInputCloud_->points[i].getVector3fMap();
            pInputCloudWithNormals->points[kept].getRGBVector3i() = pInputCloud_->points[i].getRGBVector3i();
            pInputCloudWithNormals->points[kept].getNormalVector3fMap() = pSceneNormals_->points[i].getNormalVector3fMap();
            kept++;
        }
    }
    pInputCloudWithNormals->points.resize(kept);

    float VOXEL_SIZE_ICP_ = res;
    pcl::VoxelGrid<pcl::PointXYZRGBNormal> voxel_grid_icp;
    voxel_grid_icp.setInputCloud (pInputCloudWithNormals);
    voxel_grid_icp.setDownsampleAllData(true);
    voxel_grid_icp.setLeafSize (VOXEL_SIZE_ICP_, VOXEL_SIZE_ICP_, VOXEL_SIZE_ICP_);
    voxel_grid_icp.filter (*pInputCloudWithNormals_ds);

    pInputCloud_ds->points.resize(pInputCloudWithNormals_ds->points.size());
    pInputNormals_ds->points.resize(pInputCloudWithNormals_ds->points.size());
    for(size_t i=0; i<pInputCloudWithNormals_ds->points.size(); i++)
    {
        pInputCloud_ds->points[i].getVector3fMap() = pInputCloudWithNormals_ds->points[i].getVector3fMap();
        if (!pcl_isfinite(pInputCloud_ds->points[i].x) || !pcl_isfinite(pInputCloud_ds->points[i].y) || !pcl_isfinite(pInputCloud_ds->points[i].z))
            std::cout << "Point is infinity." << std::endl;
        pInputCloud_ds->points[i].getRGBVector3i() = pInputCloudWithNormals_ds->points[i].getRGBVector3i();
        pInputNormals_ds->points[i].getNormalVector3fMap() = pInputCloudWithNormals_ds->points[i].getNormalVector3fMap();
    }

    std::cout << "cloud is organized:" << pInputCloud_ds->isOrganized() << std::endl;

    {
        pcl::ScopeTime t("finding planes...");
        //compute planes

        faat_pcl::MultiPlaneSegmentation<PointT> mps;
        mps.setInputCloud(pInputCloud_);
        mps.setMinPlaneInliers(1000);
        mps.setResolution(res);
        mps.setMergePlanes(true);
        mps.segment(false);
        planes_found_ = mps.getModels();
        std::cout << "Number of planes found in the scene:" << planes_found_.size() << std::endl;
    }

//    pcl::NormalEstimationOMP<PointT, pcl::Normal> ne;
//    ne.setRadiusSearch(0.02f);
//    ne.setInputCloud (pInputCloud_ds);
//    ne.compute (*pInputNormals_ds);

    typename faat_pcl::recognition::GHVCudaWrapper<PointT> ghv;
    ghv.setInlierThreshold(0.01f);
    ghv.setOutlierWewight(3.f);
    ghv.setClutterWeight(5.f);
    ghv.setclutterRadius(0.03f);
    ghv.setColorSigmas(0.5f, 0.5f);

    std::vector<typename pcl::PointCloud<PointT>::ConstPtr> aligned_models;
    std::vector<pcl::PointCloud<pcl::Normal>::ConstPtr> aligned_normals;
    std::vector<pcl::PointCloud<pcl::PointXYZL>::Ptr> aligned_smooth_faces;

    aligned_models.resize (models_->size ());
    aligned_smooth_faces.resize (models_->size ());
    aligned_normals.resize (models_->size ());

    std::map<std::string, int> id_to_model_clouds;
    std::map<std::string, int>::iterator it;
    std::vector<Eigen::Matrix4f> transformations;
    std::vector<int> transforms_to_models;
    transforms_to_models.resize(models_->size());
    transformations.resize(models_->size());

    int individual_models = 0;

    for (size_t kk = 0; kk < models_->size (); kk++)
    {

        int pos = 0;
        it = id_to_model_clouds.find(models_->at(kk)->id_);
        if(it == id_to_model_clouds.end())
        {
            //not included yet
            ConstPointInTPtr model_cloud = models_->at (kk)->getAssembled (res);
            pcl::PointCloud<pcl::Normal>::ConstPtr normal_cloud = models_->at (kk)->getNormalsAssembled (res);
            aligned_models[individual_models] = model_cloud;
            aligned_normals[individual_models] = normal_cloud;
            pos = individual_models;

            id_to_model_clouds.insert(std::make_pair(models_->at(kk)->id_, individual_models));

            individual_models++;
        }
        else
        {
            pos = it->second;
        }

        transformations[kk] = transforms_->at(kk);
        transforms_to_models[kk] = pos;
    }

    aligned_models.resize(individual_models);
    aligned_normals.resize(individual_models);
    std::cout << "aligned models size:" << aligned_models.size() << " " << models_->size() << std::endl;

    ghv.setSceneCloud(pInputCloud_ds);
    ghv.setSceneNormals(pInputNormals_ds);
    ghv.setOcclusionCloud(pOcclusionCloud);
    ghv.addModelNormals(aligned_normals);
    ghv.addModels(aligned_models, transformations, transforms_to_models);

    if(add_planes_)
        ghv.addPlanarModels(planes_found_);

    ghv.verify();

    float t_cues = ghv.getCuesComputationTime();
    float t_opt = ghv.getOptimizationTime();
    int num_p = ghv.getNumberOfVisiblePoints();
    std::vector<bool> mask_hv_with_planes = ghv.getSolution();


//    std::vector<int> coming_from;
//    coming_from.resize(transforms_.size() + planes_found_.size());
//    for(size_t j=0; j < transforms_.size(); j++)
//        coming_from[j] = 0;

//    for(size_t j=0; j < planes_found_.size(); j++)
//        coming_from[transforms_.size() + j] = 1;

    mask_hv.resize(transforms_->size ());
    for (size_t j = 0; j < transforms_->size (); j++)
    {
//        if(coming_from[j] == 1) // it is a plane - therefore discard
//        {
//////            mask_hv[j]=0;
////            if(j < aligned_models_.size())
////            {
////                std::cerr << "Model plane not at the end of hypotheses vector!! Check this part of code again!" << std::endl;
////            }
//            continue;
//        }
        mask_hv[j] = mask_hv_with_planes[j];
    }
    return true;
}




bool Recognizer::do_shot() const
{
    return do_shot_;
}

void Recognizer::setDo_shot(bool do_shot)
{
    do_shot_ = do_shot;
}
bool Recognizer::recognize ()
{

    std::vector<bool> mask_hv;

    constructHypotheses();

    //    std::vector<std::string> full_model_path (model_ids_.size());
//    for(size_t model_id=0; model_id < model_ids_.size(); model_id++)
//    {
//        std::stringstream model_name;
//        model_name << models_dir_ << model_ids_[model_id];
//        full_model_path[model_id] = model_name.str();
//    }

    setModelsAndTransforms(*models_, *transforms_);
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

    multi_recog_.reset (new faat_pcl::rec_3d_framework::MultiRecognitionPipeline<PointT>);

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

    if(do_shot_)
    {
        std::string idx_flann_fn = "shot_flann.idx";
        std::string desc_name = "shot";
        bool use_cache = true;
        float test_sampling_density = 0.01f;

        //configure mesh source
        typedef pcl::PointXYZRGB PointT;
        boost::shared_ptr < faat_pcl::rec_3d_framework::RegisteredViewsSource<pcl::PointXYZRGBNormal, PointT, PointT>
                > mesh_source (new faat_pcl::rec_3d_framework::RegisteredViewsSource<pcl::PointXYZRGBNormal, pcl::PointXYZRGB, pcl::PointXYZRGB>);
        mesh_source->setPath (models_dir_);
        mesh_source->setModelStructureDir (sift_structure_);
        mesh_source->setLoadViews(false);
        mesh_source->generate (training_dir_shot_);

        boost::shared_ptr < faat_pcl::rec_3d_framework::Source<PointT> > cast_source;
        cast_source = boost::static_pointer_cast<faat_pcl::rec_3d_framework::RegisteredViewsSource<pcl::PointXYZRGBNormal, PointT, PointT> > (mesh_source);

        boost::shared_ptr<faat_pcl::rec_3d_framework::UniformSamplingExtractor<PointT> > uniform_keypoint_extractor ( new faat_pcl::rec_3d_framework::UniformSamplingExtractor<PointT>);
        uniform_keypoint_extractor->setSamplingDensity (0.01f);
        uniform_keypoint_extractor->setFilterPlanar (true);
        uniform_keypoint_extractor->setMaxDistance(1.5f);
        uniform_keypoint_extractor->setThresholdPlanar(0.1);

        boost::shared_ptr<faat_pcl::rec_3d_framework::KeypointExtractor<PointT> > keypoint_extractor;
        keypoint_extractor = boost::static_pointer_cast<faat_pcl::rec_3d_framework::KeypointExtractor<PointT> > (uniform_keypoint_extractor);

        boost::shared_ptr<faat_pcl::rec_3d_framework::PreProcessorAndNormalEstimator<PointT, pcl::Normal> > normal_estimator;
        normal_estimator.reset (new faat_pcl::rec_3d_framework::PreProcessorAndNormalEstimator<PointT, pcl::Normal>);
        normal_estimator->setCMR (false);
        normal_estimator->setDoVoxelGrid (true);
        normal_estimator->setRemoveOutliers (false);
        normal_estimator->setValuesForCMRFalse (0.003f, 0.02f);

        boost::shared_ptr<faat_pcl::rec_3d_framework::SHOTLocalEstimationOMP<PointT, pcl::Histogram<352> > > estimator;
        estimator.reset (new faat_pcl::rec_3d_framework::SHOTLocalEstimationOMP<PointT, pcl::Histogram<352> >);
        estimator->setNormalEstimator (normal_estimator);
        estimator->addKeypointExtractor (keypoint_extractor);
        estimator->setSupportRadius (0.04f);
        estimator->setAdaptativeMLS (false);

        boost::shared_ptr<faat_pcl::rec_3d_framework::LocalEstimator<PointT, pcl::Histogram<352> > > cast_estimator;
        cast_estimator = boost::dynamic_pointer_cast<faat_pcl::rec_3d_framework::LocalEstimator<PointT, pcl::Histogram<352> > > (estimator);

        boost::shared_ptr<faat_pcl::rec_3d_framework::LocalRecognitionPipeline<flann::L1, PointT, pcl::Histogram<352> > > local;
        local.reset(new faat_pcl::rec_3d_framework::LocalRecognitionPipeline<flann::L1, PointT, pcl::Histogram<352> > (idx_flann_fn));
        local->setDataSource (cast_source);
        local->setTrainingDir (training_dir_shot_);
        local->setDescriptorName (desc_name);
        local->setFeatureEstimator (cast_estimator);
        local->setCGAlgorithm (cast_cg_alg);
        local->setKnn(knn_shot_);
        local->setUseCache (use_cache);
        local->setThresholdAcceptHyp (1);
        uniform_keypoint_extractor->setSamplingDensity (test_sampling_density);
        local->setICPIterations (0);
        local->setKdtreeSplits (128);
        local->initialize (false);
        local->setMaxDescriptorDistance(std::numeric_limits<float>::infinity());
        uniform_keypoint_extractor->setMaxDistance(1.5f);

        boost::shared_ptr<faat_pcl::rec_3d_framework::Recognizer<PointT> > cast_recog;
        cast_recog = boost::static_pointer_cast<faat_pcl::rec_3d_framework::LocalRecognitionPipeline<flann::L1, PointT, pcl::Histogram<352> > > (local);
        multi_recog_->addRecognizer(cast_recog);
    }

    multi_recog_->setSaveHypotheses(true);
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
