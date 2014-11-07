#ifndef SINGLEVIEW_OBJECT_RECOGNIZER_H
#define SINGLEVIEW_OBJECT_RECOGNIZER_H

#define EIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET

#include <pcl/apps/dominant_plane_segmentation.h>
#include <pcl/common/common.h>
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions.h>
#include <v4r/ORFramework/opencv_sift_local_estimator.h>
#include <v4r/ORFramework/sift_local_estimator.h>
#include <v4r/ORFramework/shot_local_estimator.h>
#include <v4r/ORFramework/shot_local_estimator_omp.h>
//#include <faat_pcl/3d_rec_framework/feature_wrapper/global/color_ourcvfh_estimator.h>
#include <v4r/ORFramework/organized_color_ourcvfh_estimator.h>
#include <v4r/ORFramework/ourcvfh_estimator.h>
//#include <faat_pcl/3d_rec_framework/pc_source/model_only_source.h>
#include <v4r/ORFramework/registered_views_source.h>
#include <v4r/ORFramework/partial_pcd_source.h>
#include <v4r/ORFramework/global_nn_recognizer_cvfh.h>
#include <v4r/ORFramework/local_recognizer.h>
#include <v4r/ORFramework/multi_pipeline_recognizer.h>
#include <v4r/ORFramework/multiplane_segmentation.h>
#include <v4r/ORFramework/metrics.h>
#include <v4r/ORRecognition/graph_geometric_consistency.h>
//#include <faat_pcl/recognition/hv/hv_cuda_wrapper.h>
#include <v4r/ORRecognition/ghv.h>
#include <v4r/ORRegistration/visibility_reasoning.h>
#include <v4r/ORUtils/miscellaneous.h>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/lexical_cast.hpp>
#include "segmenter.h"
#include "recognition_srv_definitions/recognize.h"
#include "boost_graph_extension.h"


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

    int icp_iterations_;
    int icp_type_;
    float icp_voxel_size_;

    std::map<std::string, faat_pcl::rec_3d_framework::ObjectHypothesis<PointT> > hypotheses_;
    boost::shared_ptr< pcl::PointCloud<PointT> > pKeypointsMultipipe_;
    pcl::PointIndices keypointIndices_;
    cv::Ptr<SiftGPU> sift_;
    pcl::PointCloud<PointT>::Ptr pInputCloud_;
    pcl::PointCloud<pcl::Normal>::Ptr pSceneNormals_;
    boost::shared_ptr < std::vector<ModelTPtr> > models_;
    boost::shared_ptr < std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > > transforms_;
    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > transforms_verified_;
    std::vector<std::string> model_ids_verified_;
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr> aligned_models_;
    std::vector<pcl::PointCloud<pcl::Normal>::ConstPtr> aligned_normals_;
    std::vector<pcl::PointCloud<pcl::PointXYZL>::Ptr> aligned_smooth_faces_;
    std::vector<std::string> model_ids_;
    std::vector<faat_pcl::PlaneModel<PointT> > planes_found_;
    std::vector<pcl::PointCloud<PointT>::Ptr> verified_planes_;
//    boost::shared_ptr < faat_pcl::rec_3d_framework::ModelOnlySource<pcl::PointXYZRGBNormal, pcl::PointXYZRGB>
//            > model_only_source_;

    bool add_planes_;
    int knn_shot_;

    struct hv_params{
            float resolution_;
            float inlier_threshold_;
            float radius_clutter_;
            float regularizer_;
            float clutter_regularizer_;
            float occlusion_threshold_;
            int optimizer_type_;
            float color_sigma_l_;
            float color_sigma_ab_;
    }hv_params_;

    struct cg_params{
        int cg_size_threshold_;
        float cg_size_;
        float ransac_threshold_;
        float dist_for_clutter_factor_;
        int max_taken_;
        float max_time_for_cliques_computation_;
        float dot_distance_;
    }cg_params_;

#ifdef SOC_VISUALIZE
    boost::shared_ptr<pcl::visualization::PCLVisualizer> vis_;
    int v1_,v2_, v3_;
#endif


public:

    Recognizer ()
    {
        //default values
        do_sift_ = true;
        do_shot_ = false;
        do_ourcvfh_ = false;

        icp_iterations_ = 0;
        icp_type_ = 1;
        icp_voxel_size_ = 0.005;

        hv_params_.resolution_ = 0.005f;
        hv_params_.inlier_threshold_ = 0.015;
        hv_params_.radius_clutter_ = 0.03;
        hv_params_.regularizer_ = 3;
        hv_params_.clutter_regularizer_ = 5;
        hv_params_.occlusion_threshold_ = 0.01;
        hv_params_.optimizer_type_ = 0;
        hv_params_.color_sigma_l_ = 0.5;
        hv_params_.color_sigma_ab_ = 0.5;

        cg_params_.cg_size_threshold_ = 3;
        cg_params_.cg_size_ = 0.015;
        cg_params_.ransac_threshold_ = 0.015;
        cg_params_.dist_for_clutter_factor_ = 0;
        cg_params_.max_taken_ = 2;
        cg_params_.max_time_for_cliques_computation_ = 100;
        cg_params_.dot_distance_ = 0.2;

        add_planes_ = true;
        knn_shot_ = 1;

        pInputCloud_.reset(new pcl::PointCloud<PointT>);
        pSceneNormals_.reset(new pcl::PointCloud<pcl::Normal>);

//        model_only_source_.reset (new faat_pcl::rec_3d_framework::ModelOnlySource<pcl::PointXYZRGBNormal, pcl::PointXYZRGB>);


#ifdef SOC_VISUALIZE
        vis_.reset (new pcl::visualization::PCLVisualizer ("classifier visualization"));
        vis_->createViewPort(0,0,0.33,1.f, v1_);
        vis_->createViewPort(0.33,0,0.66,1.f, v2_);
        vis_->createViewPort(0.66,0,1,1.f, v3_);
#endif
    }

    void set_hv_resolution(const float res)
    {
        hv_params_.resolution_ = res;
    }

    void set_hv_inlier_threshold(const float thres)
    {
        hv_params_.inlier_threshold_ = thres;
    }

    void set_hv_radius_clutter(const float radius_clutter)
    {
        hv_params_.radius_clutter_ = radius_clutter;
    }

    void set_hv_regularizer(const float regularizer)
    {
        hv_params_.regularizer_ = regularizer;
    }

    void set_hv_clutter_regularizer (const float clutter_reg)
    {
        hv_params_.clutter_regularizer_ = clutter_reg;
    }

    void set_hv_occlusion_threshold ( const float occ_thresh)
    {
        hv_params_.occlusion_threshold_ = occ_thresh;
    }

    void set_hv_optimizer_type (const int opt_type)
    {
        hv_params_.optimizer_type_ = opt_type;
    }

    void set_hv_color_sigma_L ( const float sigma_l)
    {
        hv_params_.color_sigma_l_ = sigma_l;
    }

    void set_hv_color_sigma_AB ( const float sigma_ab)
    {
        hv_params_.color_sigma_ab_ = sigma_ab;
    }


    void set_cg_size_threshold ( const int cg_size)
    {
        cg_params_.cg_size_threshold_ = cg_size;
    }

    void set_cg_size (const float cg_size)
    {
        cg_params_.cg_size_ = cg_size;
    }

    void set_cg_ransac_threshold ( const float ransac_thresh)
    {
        cg_params_.ransac_threshold_ = ransac_thresh;
    }

    void set_cg_dist_for_clutter_factor ( const float dist_for_clutter_factor )
    {
        cg_params_.dist_for_clutter_factor_ = dist_for_clutter_factor;
    }

    void set_cg_max_taken (const int max_taken)
    {
        cg_params_.max_taken_ = max_taken;
    }

    void set_cg_max_time_for_cliques_computation (const float max_time)
    {
        cg_params_.max_time_for_cliques_computation_ = max_time;
    }

    void set_cg_dot_distance (const float dist)
    {
        cg_params_.dot_distance_ = dist;
    }


    bool recognize ();

    void initialize();

    void getVerifiedPlanes(std::vector<pcl::PointCloud<PointT>::Ptr> &planes) const
    {
        planes = verified_planes_;
    }

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

    void set_do_sift(const bool do_sift)
    {
        do_sift_ = do_sift;
    }

    void set_icp_iterations(int icp_iterations)
    {
        icp_iterations_ = icp_iterations;
    }


    void set_icp_type (const int type)
    {
        icp_type_ = type;
    }

    void set_icp_voxel_size (const float size)
    {
        icp_voxel_size_ = size;
    }

    bool do_ourcvfh() const
    {
        return do_ourcvfh_;
    }

    void set_do_ourcvfh(const bool do_ourcvfh)
    {
        do_ourcvfh_ = do_ourcvfh;
    }

    bool do_shot() const
    {
        return do_shot_;
    }

    void set_do_shot(const bool do_shot)
    {
        do_shot_ = do_shot;
    }

    cv::Ptr<SiftGPU> getSift() const
    {
        return sift_;
    }

    void set_sift(const cv::Ptr<SiftGPU> &value)
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
        aligned_normals_.resize(models.size());
        model_ids_.resize(models.size());
        *transforms_ = transforms;
        *models_ = models;
        aligned_smooth_faces_.resize (models_->size ());

        for(size_t i=0; i<models.size(); i++)
        {
//            ModelTPtr m_with_faces;
//            model_only_source_->getModelById(models.at(i)->id_, m_with_faces);

            ConstPointInTPtr model_cloud = models.at(i)->getAssembled (hv_params_.resolution_);
            typename pcl::PointCloud<PointT>::Ptr model_aligned (new pcl::PointCloud<PointT>);
            pcl::transformPointCloud (*model_cloud, *model_aligned, transforms[i]);
            aligned_models_[i] = model_aligned;
            model_ids_[i] = models.at(i)->id_;

//            pcl::PointCloud<pcl::PointXYZL>::Ptr faces = models.at(i)->getAssembledSmoothFaces(hv_params_.resolution_);
//            pcl::PointCloud<pcl::PointXYZL>::Ptr faces_aligned(new pcl::PointCloud<pcl::PointXYZL>);
//            pcl::transformPointCloud (*faces, *faces_aligned, transforms[i]);
//            aligned_smooth_faces_ [i] = faces_aligned;

            pcl::PointCloud<pcl::Normal>::ConstPtr normal_cloud_const = models.at(i)->getNormalsAssembled (hv_params_.resolution_);
            pcl::PointCloud<pcl::Normal>::Ptr normal_cloud(new pcl::PointCloud<pcl::Normal>(*normal_cloud_const) );

            const Eigen::Matrix3f rot   = transforms_->at(i).block<3, 3> (0, 0);
//            const Eigen::Vector3f trans = transforms_->at(i).block<3, 1> (0, 3);
            for(size_t jj=0; jj < normal_cloud->points.size(); jj++)
            {
                const pcl::Normal norm_pt = normal_cloud->points[jj];
                normal_cloud->points[jj].getNormalVector3fMap() = rot * norm_pt.getNormalVector3fMap();
            }
            aligned_normals_[i] = normal_cloud;
        }
    }

    void setInputCloud(const pcl::PointCloud<PointT>::ConstPtr pInputCloud, const pcl::PointCloud<pcl::Normal>::ConstPtr pSceneNormals)
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

    void poseRefinement()
    {
        multi_recog_->getPoseRefinement(models_, transforms_);
    }

    bool hypothesesVerification(std::vector<bool> &mask_hv);
//    bool hypothesesVerificationGpu(std::vector<bool> &mask_hv);

    bool multiplaneSegmentation();

    void visualizeHypotheses();

    void constructHypotheses();

    void preFilterWithFSV(const pcl::PointCloud<PointT>::ConstPtr scene_cloud, std::vector<float> &fsv);

    void constructHypothesesFromFeatureMatches(std::map < std::string,faat_pcl::rec_3d_framework::ObjectHypothesis<PointT> > hypothesesInput,
                                               pcl::PointCloud<PointT>::Ptr pKeypoints,
                                               pcl::PointCloud<pcl::Normal>::Ptr pKeypointNormals,
                                               std::vector<Hypothesis<PointT> > &hypothesesOutput,
                                               std::vector <pcl::Correspondences> &corresp_clusters);
};

#endif //SINGLEVIEW_OBJECT_RECOGNIZER_H
