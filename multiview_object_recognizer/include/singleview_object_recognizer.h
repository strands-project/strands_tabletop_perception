#ifndef SINGLEVIEW_OBJECT_RECOGNIZER_H
#define SINGLEVIEW_OBJECT_RECOGNIZER_H

#define EIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET

#include <pcl/apps/dominant_plane_segmentation.h>
#include <pcl/common/common.h>
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions.h>
#include <faat_pcl/3d_rec_framework/feature_wrapper/local/image/opencv_sift_local_estimator.h>
#include <faat_pcl/3d_rec_framework/feature_wrapper/local/image/sift_local_estimator.h>
#include <faat_pcl/3d_rec_framework/feature_wrapper/local/shot_local_estimator.h>
#include <faat_pcl/3d_rec_framework/feature_wrapper/local/shot_local_estimator_omp.h>
//#include <faat_pcl/3d_rec_framework/feature_wrapper/global/color_ourcvfh_estimator.h>
#include <faat_pcl/3d_rec_framework/feature_wrapper/global/organized_color_ourcvfh_estimator.h>
#include <faat_pcl/3d_rec_framework/feature_wrapper/global/ourcvfh_estimator.h>
#include <faat_pcl/3d_rec_framework/pc_source/registered_views_source.h>
#include <faat_pcl/3d_rec_framework/pc_source/partial_pcd_source.h>
#include <faat_pcl/3d_rec_framework/pipeline/global_nn_recognizer_cvfh.h>
#include <faat_pcl/3d_rec_framework/pipeline/local_recognizer.h>
#include <faat_pcl/3d_rec_framework/pipeline/multi_pipeline_recognizer.h>
#include <faat_pcl/3d_rec_framework/segmentation/multiplane_segmentation.h>
#include <faat_pcl/3d_rec_framework/utils/metrics.h>
#include <faat_pcl/recognition/cg/graph_geometric_consistency.h>
#include <faat_pcl/recognition/hv/ghv_cuda_wrapper.h>
#include <faat_pcl/recognition/hv/hv_go_1.h>
#include <faat_pcl/registration/visibility_reasoning.h>
#include <faat_pcl/utils/miscellaneous.h>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/lexical_cast.hpp>
#include "segmenter.h"
#include "recognition_srv_definitions/recognize.h"


struct camPosConstraints
{
    bool
    operator() (const Eigen::Vector3f & pos) const
    {
        if (pos[2] > 0)
            return true;

        return false;
    }
    ;
};

class Recognizer
{
private:
    typedef pcl::PointXYZRGB PointT;
    typedef faat_pcl::rec_3d_framework::Model<PointT> ModelT;
    typedef boost::shared_ptr<ModelT> ModelTPtr;
    typedef typename pcl::PointCloud<PointT>::Ptr PointInTPtr;
    typedef typename pcl::PointCloud<PointT>::ConstPtr ConstPointInTPtr;

    boost::shared_ptr<faat_pcl::rec_3d_framework::MultiRecognitionPipeline<PointT> > multi_recog_;
    std::string models_dir_;
    std::string training_dir_sift_;
    std::string training_dir_shot_;
    std::string sift_structure_;
    std::string training_dir_ourcvfh_;
    bool do_sift_;
    bool do_shot_;
    bool do_ourcvfh_;
    double chop_at_z_;
    int icp_iterations_;
//    std::vector<std::string> text_3d_;
    std::map<std::string, faat_pcl::rec_3d_framework::ObjectHypothesis<PointT> > hypotheses_;
    boost::shared_ptr< pcl::PointCloud<PointT> > pKeypointsMultipipe_;
    pcl::PointIndices keypointIndices_;
    int cg_size_;
    bool ignore_color_;
    cv::Ptr<SiftGPU> sift_;
    pcl::PointCloud<PointT>::Ptr pInputCloud_;
    pcl::PointCloud<pcl::Normal>::Ptr pSceneNormals_;
    boost::shared_ptr < std::vector<ModelTPtr> > models_;
    boost::shared_ptr < std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > > transforms_;
    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > transforms_verified_;
    std::vector<std::string> model_ids_verified_;
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr> aligned_models_;
    std::vector<std::string> model_ids_;
    std::vector<faat_pcl::PlaneModel<PointT> > planes_found_;
    float go_resolution_;
    bool add_planes_;
    int knn_shot_;

#ifdef SOC_VISUALIZE
    boost::shared_ptr<pcl::visualization::PCLVisualizer> vis_;
    int v1_,v2_, v3_;
#endif


public:

    Recognizer ()
    {
        //default values
        chop_at_z_ = 1.5;
        do_sift_ = true;
        do_shot_ = false;
        do_ourcvfh_ = false;
        icp_iterations_ = 0;
        cg_size_ = 3;
        go_resolution_ = 0.005f;
        add_planes_ = true;
        knn_shot_ = 1;


        pInputCloud_.reset(new pcl::PointCloud<PointT>);
        pSceneNormals_.reset(new pcl::PointCloud<pcl::Normal>);

#ifdef SOC_VISUALIZE
        vis_.reset (new pcl::visualization::PCLVisualizer ("classifier visualization"));
        vis_->createViewPort(0,0,0.33,1.f, v1_);
        vis_->createViewPort(0.33,0,0.66,1.f, v2_);
        vis_->createViewPort(0.66,0,1,1.f, v3_);
#endif
    }

    bool recognize ();

    void initialize();

    std::string training_dir_sift() const
    {
        return training_dir_sift_;
    }

    void setTraining_dir_sift(const std::string &training_dir_sift)
    {
        training_dir_sift_ = training_dir_sift;
    }

    std::string training_dir_shot() const
    {
        return training_dir_shot_;
    }

    void setTraining_dir_shot(const std::string &training_dir_shot)
    {
        training_dir_shot_ = training_dir_shot;
    }

    std::string models_dir() const
    {
        return models_dir_;
    }

    void setModels_dir(const std::string &models_dir)
    {
        models_dir_ = models_dir;
    }

    std::string sift_structure() const
    {
        return sift_structure_;
    }

    void setSift_structure(const std::string &sift_structure)
    {
        sift_structure_ = sift_structure;
    }

    std::string training_dir_ourcvfh() const
    {
        return training_dir_ourcvfh_;
    }

    void setTraining_dir_ourcvfh(const std::string &training_dir_ourcvfh)
    {
        training_dir_ourcvfh_ = training_dir_ourcvfh;
    }

    bool do_sift() const
    {
        return do_sift_;
    }

    void setDo_sift(bool do_sift)
    {
        do_sift_ = do_sift;
    }

    double chop_at_z() const
    {
        return chop_at_z_;
    }

    void setChop_at_z(double chop_at_z)
    {
        chop_at_z_ = chop_at_z;
    }

    int icp_iterations() const
    {
        return icp_iterations_;
    }

    void setIcp_iterations(int icp_iterations)
    {
        icp_iterations_ = icp_iterations;
    }

    bool ignore_color() const
    {
        return ignore_color_;
    }

    void setIgnore_color(bool ignore_color)
    {
        ignore_color_ = ignore_color;
    }

    bool do_ourcvfh() const
    {
        return do_ourcvfh_;
    }

    void setDo_ourcvfh(bool do_ourcvfh)
    {
        do_ourcvfh_ = do_ourcvfh;
    }

    cv::Ptr<SiftGPU> getSift() const
    {
        return sift_;
    }

    void setSift(const cv::Ptr<SiftGPU> &value)
    {
        sift_ = value;
    }

    void getSavedHypotheses(std::map<std::string, faat_pcl::rec_3d_framework::ObjectHypothesis<PointT> > & hypotheses) const
    {
        hypotheses = hypotheses_;
    }

    void getKeypointsMultipipe (boost::shared_ptr<pcl::PointCloud<PointT> > &pKeypointsMultipipe ) const
    {
        pKeypointsMultipipe = pKeypointsMultipipe_;
    }

    void getKeypointIndices(pcl::PointIndices &keypointIndices) const
    {
        keypointIndices.header = keypointIndices_.header;
        keypointIndices.indices = keypointIndices_.indices;
    }

    template <template<class > class Distance, typename FeatureT>
    void setISPK(typename pcl::PointCloud<FeatureT>::Ptr & signatures, PointInTPtr & p, pcl::PointIndices & keypoint_indices, size_t feature_type)
    {
        multi_recog_->setISPK<Distance, FeatureT>(signatures, p, keypoint_indices, feature_type);
    }

    void getModelsAndTransforms(std::vector<std::string> &models_verified, std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > &transforms_verified) const
    {
        models_verified = model_ids_verified_;
        transforms_verified = transforms_verified_;
    }

    void getAllHypotheses(std::vector<std::string> &models, std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> &transforms) const
    {
        models = model_ids_;
        transforms = *transforms_;
    }

    void setModelsAndTransforms(const std::vector<std::string> &models, const std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> &transforms)
    {
        aligned_models_.resize(models.size());
        model_ids_.resize(models.size());
        *transforms_ = transforms;

        models_->clear();       // NOT IMPLEMENTED YET!!

        for(size_t i=0; i<models.size(); i++)
        {
            boost::filesystem::path modelpath(models[i]);
            model_ids_[i] =  modelpath.filename().string();
            PointInTPtr pModelPCl ( new pcl::PointCloud<pcl::PointXYZRGB> );
            PointInTPtr pModelPClTransformed ( new pcl::PointCloud<pcl::PointXYZRGB> );
            PointInTPtr pModelPCl2 ( new pcl::PointCloud<pcl::PointXYZRGB> );
            pcl::io::loadPCDFile ( models[i], * ( pModelPCl ) );

            pcl::transformPointCloud ( *pModelPCl, *pModelPClTransformed, transforms[i] );

            pcl::VoxelGrid<pcl::PointXYZRGB> sor;
            float leaf = 0.005f;
            sor.setLeafSize ( leaf, leaf, leaf );
            sor.setInputCloud ( pModelPClTransformed );
            sor.filter ( *pModelPCl2 );

            aligned_models_[i] = pModelPCl2;
            //models_ = models;
            //transforms_ = transforms;
        }
    }

    void setModelsAndTransforms(const std::vector<ModelTPtr> &models, const std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> &transforms)
    {
        aligned_models_.resize(models.size());
        model_ids_.resize(models.size());
        *transforms_ = transforms;
        *models_ = models;

        for(size_t i=0; i<models.size(); i++)
        {
            ConstPointInTPtr model_cloud = models.at (i)->getAssembled (go_resolution_);
            typename pcl::PointCloud<PointT>::Ptr model_aligned (new pcl::PointCloud<PointT>);
            pcl::transformPointCloud (*model_cloud, *model_aligned, transforms[i]);
            aligned_models_[i] = model_aligned;
            model_ids_[i] = models.at(i)->id_;
        }
    }

    void setInputCloud(const pcl::PointCloud<PointT>::ConstPtr pInputCloud, const pcl::PointCloud<pcl::Normal>::ConstPtr pSceneNormals = new pcl::PointCloud<pcl::Normal>())
    {
        pcl::copyPointCloud(*pInputCloud, *pInputCloud_);
        pcl::copyPointCloud(*pSceneNormals, *pSceneNormals_);
        model_ids_verified_.clear();
        transforms_verified_.clear();
        aligned_models_.clear();
        model_ids_.clear();

        if(transforms_)
            transforms_->clear();

//        if(models_)
//            models_->clear();
    }

    void poseRefinement(boost::shared_ptr<std::vector<ModelTPtr> > models,
                        boost::shared_ptr<std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > > transforms)
    {
        multi_recog_->getPoseRefinement(models, transforms);
    }

    bool hypothesesVerification(std::vector<bool> &mask_hv);
    bool hypothesesVerificationGpu(std::vector<bool> &mask_hv);

    bool multiplaneSegmentation();

    void visualizeHypotheses();

    void constructHypotheses();
    bool do_shot() const;
    void setDo_shot(bool do_shot);
};

#endif //SINGLEVIEW_OBJECT_RECOGNIZER_H
