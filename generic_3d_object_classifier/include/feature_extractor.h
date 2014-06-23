#ifndef FEATURE_EXTRACTOR_H
#define FEATURE_EXTRACTOR_H


#define EIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET

#include <cv.h>
#include <opencv2/nonfree/features2d.hpp>
#include <iostream>
#include <vector>
#include <string>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/io.h>

#include <faat_pcl/3d_rec_framework/feature_wrapper/global/esf_estimator.h>
#include <faat_pcl/3d_rec_framework/feature_wrapper/local/image/sift_local_estimator.h>
#include <faat_pcl/3d_rec_framework/feature_wrapper/local/shot_local_estimator_omp.h>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT>::Ptr PointInTPtr;
typedef pcl::Histogram<128> SIFTFeatureT;
typedef pcl::PointCloud<SIFTFeatureT>::Ptr SIFTFeatureTPtr;
typedef pcl::Histogram<352> SHOTFeatureT;
typedef pcl::PointCloud<SHOTFeatureT>::Ptr SHOTFeatureTPtr;
typedef pcl::ESFSignature640 ESFFeatureT;
typedef pcl::PointCloud<ESFFeatureT>::Ptr ESFFeatureTPtr;

class FeatureExtractor
{
private:
    boost::shared_ptr < faat_pcl::rec_3d_framework::SIFTLocalEstimation<PointT, SIFTFeatureT > >
        sift_estimator_;
    boost::shared_ptr < faat_pcl::rec_3d_framework::ESFEstimation<PointT, ESFFeatureT > > esf_estimator_;
    boost::shared_ptr<faat_pcl::rec_3d_framework::SHOTLocalEstimationOMP<PointT, pcl::Histogram<352> > > shot_estimator_;

public:
    FeatureExtractor()
    {
        sift_estimator_.reset (new faat_pcl::rec_3d_framework::SIFTLocalEstimation<PointT, SIFTFeatureT >());
        esf_estimator_.reset (new faat_pcl::rec_3d_framework::ESFEstimation<PointT, ESFFeatureT >());
        shot_estimator_.reset (new faat_pcl::rec_3d_framework::SHOTLocalEstimationOMP<PointT, pcl::Histogram<352> >);

        //sXYZ_LUT[4000] = {- 1};
        //sRGB_LUT[256] = {- 1};
    }

    void computeSIFTFeatures(const cv::Mat &inputX, cv::Mat &FeatureVector, size_t &num_dimensions);
    void computeOpenCVSIFTFeatures(const cv::Mat &inputX, cv::Mat &FeatureVector, size_t &num_dimensions);
    void computeSIFTFeatures(const PointInTPtr &pInputX, const pcl::PointIndices &indices, cv::Mat &FeatureMatrix, size_t &num_dimensions);
    void computeESFFeatures(const PointInTPtr &pInputX, const pcl::PointIndices &indices, cv::Mat &FeatureMatrix, size_t &num_dimensions);
    void computeESFFeatures(const PointInTPtr &pInputX, cv::Mat &FeatureMatrix, size_t &num_dimensions);
    void computeSHOTFeatures(const PointInTPtr &pInputX, const pcl::PointIndices &indices, cv::Mat &FeatureMatrix, size_t &num_dimensions);
    void computeCustomFeatures(const PointInTPtr &pInputX, const pcl::PointIndices &indices, cv::Mat &FeatureMatrix, size_t &num_dimensions);
    void computeCustomFeatures(const cv::Mat &inputX, cv::Mat &FeatureMatrix, size_t &num_dimensions);
    void computeKahanMeanAndStd(const std::vector<float> data_v, double &mean, double &std);
};

class MyColorConverter
{
    static float sRGB_LUT[256];
    static float sXYZ_LUT[4000];

public:
    static void RGB2CIELAB (const unsigned char R, const unsigned char G, const unsigned char B, float &L, float &A,float &B2);
    static void convertBGRtoLAB(const cv::Mat_<cv::Vec3b> &im_bgr, cv::Mat_<cv::Vec3d> &im_lab);
};

#endif //FEATURE_EXTRACTOR_H
