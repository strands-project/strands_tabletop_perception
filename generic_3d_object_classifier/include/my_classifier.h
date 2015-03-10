#ifndef MY_CLASSIFIER_H
#define MY_CLASSIFIER_H

//#define EIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET

#include <vector>
#include <cv.h>
#include <pcl/common/common.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <v4r/ORRecognition/kmeans.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include "feature_extractor.h"

#include <v4r/ORFramework/unregistered_views_source.h>
#include <v4r/ORFramework/source2d.h>
#include <v4r/utils/filesystem_utils.h>

#include <v4r/SVM/svmWrapper.h>

#include "visual_codebook.h"
//#include <v4r/svm/SVMTrainModel.h>
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT>::Ptr PointInTPtr;
typedef faat_pcl::rec_3d_framework::Model<PointT> ModelT;
typedef faat_pcl::rec_3d_framework::Model2D Model2DT;
typedef boost::shared_ptr<ModelT> ModelTPtr;
typedef boost::shared_ptr<Model2DT> Model2DTPtr;

class MyClassifier
{
private:
    pcl::PointCloud<PointT>::Ptr pInputCloud_;
    std::string models_dir_, trained_dir_;
    pcl::KdTreeFLANN<ESFFeatureT> kdtree_;
    std::string test_filename_, test_dir_;
    boost::shared_ptr < faat_pcl::rec_3d_framework::UnregisteredViewsSource  <PointT>
            > pPCSource_;
    boost::shared_ptr < faat_pcl::rec_3d_framework::Source2D> pImgSource_;
    boost::shared_ptr < faat_pcl::rec_3d_framework::Source<PointT> > cast_source_;
    boost::shared_ptr<std::vector<ModelTPtr> > models3D_;
    boost::shared_ptr<std::vector<Model2DTPtr> > models2D_;
    boost::shared_ptr<FeatureExtractor> feature_extractor_;
    //pcl::PointCloud<ESFFeatureT>::Ptr esf_signatures_ ;

    std::map<std::string, size_t> class_map_;
    boost::shared_ptr <VisualCodebook > siftCodebook_;
    bool force_retrain_;
    size_t num_classes_;
    boost::shared_ptr <svm::svmWrapper> pSVM;
    size_t features3d_;
    size_t features2d_;

public:

    enum FeatureTypes_2D
    {
      SIFT_2D = 0x01, // 00000001
      CUSTOM_2D  = 0x02  // 00000010
    };
    enum FeatureTypes_3D
    {
      SIFT_3D = 0x01, // 00000001
      ESF_3D = 0x02, // 00000010
      SHOT_3D  = 0x04, // 00000100
      CUSTOM_3D  = 0x08  // 00001000
    };

    MyClassifier()
    {
        //num_classes_ = 0;
        trained_dir_ = "";
        force_retrain_ = false;
        pInputCloud_.reset(new pcl::PointCloud<PointT>());
        models_dir_ = "/home/thomas/data/Cat50_TestDB_small/pcd_binary";
        siftCodebook_.reset(new VisualCodebook());
        pImgSource_.reset(new faat_pcl::rec_3d_framework::Source2D ());
        pPCSource_.reset(new faat_pcl::rec_3d_framework::UnregisteredViewsSource  <PointT>());
        feature_extractor_.reset(new FeatureExtractor());
        pSVM.reset(new svm::svmWrapper());

        features2d_ = SIFT_2D;
        features3d_ = SIFT_3D;
    }

    void init();

    void setInputCloud(const pcl::PointCloud<PointT> &cloud);

    void trainClassifier();

    void classify();

    std::vector<std::vector<double> > readMatrixFromFile(
            const std::string filename,
            const std::string delimiter = ",");


    template <typename MTPtr>
    void assignClassToId(const boost::shared_ptr<const std::vector<MTPtr> > models);

    void testClassifier();

    void writeGlobalSignaturesToFile(
            const std::string base_dir,
            const std::vector<std::vector<std::vector<double> > > & hist_per_class_per_view_v
            );

    void setTestDir(std::string test_path)
    {
        test_dir_ = test_path;
    }

    void writeSignaturesToFile(
            const std::string base_dir,
            const std::vector<cv::Mat> & signatures_per_view_v,
            size_t class_id = 0,
            size_t num_dimensions = 0);

    void setTestFilename(std::string test_filename)
    {
        test_filename_ = test_filename;
    }

    void setTrainedDir(std::string trained_dir)
    {
        trained_dir_ = trained_dir;
    }

    void setTrainingDir(std::string model_dir)
    {
        models_dir_ = model_dir;
    }

    void setForceRetrain(bool force_retrain)
    {
        force_retrain_ = force_retrain;
    }

    void setFeatures2D(size_t features2d)
    {
        features2d_ = features2d;
    }

    void setFeatures3D(size_t features3d)
    {
        features3d_ = features3d;
    }

    void mergeGlobalDescriptors(
            const std::vector< std::vector< std::vector< std::vector< double > > > > &global_signatures_per_featureType_per_class_per_view_v,
            std::vector< std::vector< std::vector< double > > > &merged_global_signatures_per_class_per_view_v);

    void displayComputedFeatures();

};

#endif //MY_CLASSIFIER_H
