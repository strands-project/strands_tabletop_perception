#include "feature_extractor.h"
#include <pcl/visualization/pcl_visualizer.h>

float MyColorConverter::sRGB_LUT[256] = {- 1};
float MyColorConverter::sXYZ_LUT[4000] = {- 1};

//////////////////////////////////////////////////////////////////////////////////////////////
void MyColorConverter::RGB2CIELAB (const unsigned char R, const unsigned char G, const unsigned char B, float &L, float &A,float &B2)
{
  if (sRGB_LUT[0] < 0)
  {
    for (int i = 0; i < 256; i++)
    {
      float f = static_cast<float> (i) / 255.0f;
      if (f > 0.04045)
        sRGB_LUT[i] = powf ((f + 0.055f) / 1.055f, 2.4f);
      else
        sRGB_LUT[i] = f / 12.92f;
    }

    for (int i = 0; i < 4000; i++)
    {
      float f = static_cast<float> (i) / 4000.0f;
      if (f > 0.008856)
        sXYZ_LUT[i] = static_cast<float> (powf (f, 0.3333f));
      else
        sXYZ_LUT[i] = static_cast<float>((7.787 * f) + (16.0 / 116.0));
    }
  }

  float fr = sRGB_LUT[R];
  float fg = sRGB_LUT[G];
  float fb = sRGB_LUT[B];

  // Use white = D65
  const float x = fr * 0.412453f + fg * 0.357580f + fb * 0.180423f;
  const float y = fr * 0.212671f + fg * 0.715160f + fb * 0.072169f;
  const float z = fr * 0.019334f + fg * 0.119193f + fb * 0.950227f;

  float vx = x / 0.95047f;
  float vy = y;
  float vz = z / 1.08883f;

  vx = sXYZ_LUT[int(vx*4000)];
  vy = sXYZ_LUT[int(vy*4000)];
  vz = sXYZ_LUT[int(vz*4000)];

  L = 116.0f * vy - 16.0f;
  if (L > 100)
    L = 100.0f;

  A = 500.0f * (vx - vy);
  if (A > 120)
    A = 120.0f;
  else if (A <- 120)
    A = -120.0f;

  B2 = 200.0f * (vy - vz);
  if (B2 > 120)
    B2 = 120.0f;
  else if (B2<- 120)
    B2 = -120.0f;
}


void MyColorConverter::convertBGRtoLAB(const cv::Mat_<cv::Vec3b> &img_bgr, cv::Mat_<cv::Vec3d> &im_lab)
{
  im_lab = cv::Mat_<cv::Vec3b>(img_bgr.size());

  double R, G, B, r, g, b;
  double X, Y, Z, xr, yr, zr;
  double fx, fy, fz;

  double epsilon = 0.008856;  //actual CIE standard
  double kappa   = 903.3;   //actual CIE standard

  const double inv_Xr = 1./0.950456; //reference white
  //const double inv_Yr = 1./1.0;    //reference white
  const double inv_Zr = 1./1.088754; //reference white
  const double inv_255 = 1./255;
  const double inv_12 = 1./12.92;
  const double inv_1 = 1./1.055;
  const double inv_3 = 1./3.0;
  const double inv_116 = 1./116.0;

  #pragma omp parallel for private(R,G,B,r,g,b,X,Y,Z,xr,yr,zr,fx,fy,fz)
  for (int v=0; v<img_bgr.rows; v++)
  {
    for (int u=0; u<img_bgr.cols; u++)
    {
      const cv::Vec3b &rgb = img_bgr(v,u);
      cv::Vec3d &lab = im_lab(v,u);

      B = rgb[0]*inv_255;
      G = rgb[1]*inv_255;
      R = rgb[2]*inv_255;

      if(R <= 0.04045)  r = R*inv_12;
      else        r = pow((R+0.055)*inv_1,2.4);
      if(G <= 0.04045)  g = G*inv_12;
      else        g = pow((G+0.055)*inv_1,2.4);
      if(B <= 0.04045)  b = B*inv_12;
      else        b = pow((B+0.055)*inv_1,2.4);

      X = r*0.4124564 + g*0.3575761 + b*0.1804375;
      Y = r*0.2126729 + g*0.7151522 + b*0.0721750;
      Z = r*0.0193339 + g*0.1191920 + b*0.9503041;

      xr = X*inv_Xr;
      yr = Y;//*inv_Yr;
      zr = Z*inv_Zr;

      if(xr > epsilon)  fx = pow(xr, inv_3);
      else        fx = (kappa*xr + 16.0)*inv_116;
      if(yr > epsilon)  fy = pow(yr, inv_3);
      else        fy = (kappa*yr + 16.0)*inv_116;
      if(zr > epsilon)  fz = pow(zr, inv_3);
      else        fz = (kappa*zr + 16.0)*inv_116;

      lab[0] = 116.0*fy-16.0;
      lab[1] = 500.0*(fx-fy);
      lab[2] = 200.0*(fy-fz);
    }
  }
}

void FeatureExtractor::computeSIFTFeatures(const cv::Mat &inputX, cv::Mat &FeatureMatrix, size_t &num_dimensions)
{
    num_dimensions = 128;

    std::vector<SiftGPU::SiftKeypoint> keypoints;
    SIFTFeatureTPtr sift_signatures;
    sift_signatures.reset(new pcl::PointCloud<SIFTFeatureT>());
    std::vector<float> sift_scale;
    sift_estimator_->estimate(inputX, keypoints, sift_signatures, sift_scale);
    FeatureMatrix = cv::Mat(sift_signatures->points.size(), num_dimensions, CV_32F);

    for(size_t i = 0; i < sift_signatures->points.size(); i++)
    {
        for(size_t kk=0; kk < num_dimensions; kk++)
        {
            FeatureMatrix.at<float>(i,kk) = sift_signatures->points[i].histogram[kk];
        }
    }
    //std::cout << "FeatureVector: " << std::endl << FeatureVector << std::endl << std::endl;
}

void FeatureExtractor::computeOpenCVSIFTFeatures(const cv::Mat &inputX, cv::Mat &FeatureMatrix, size_t &num_dimensions)
{
    num_dimensions = 128;

    /* threshold      = 0.04;
       edge_threshold = 10.0;
       magnification  = 3.0;    */

    // SIFT feature detector and feature extractor
    cv::SiftFeatureDetector detector( 0.05, 5.0 );
    cv::SiftDescriptorExtractor extractor( 3.0 );

    /* In case of SURF, you apply the below two lines
    cv::SurfFeatureDetector detector();
    cv::SurfDescriptorExtractor extractor();
    */

    // Feature detection
    std::vector<cv::KeyPoint> keypoints;
    detector.detect( inputX, keypoints );

    // Feature descriptor computation
    cv::Mat descriptor;
    extractor.compute( inputX, keypoints, FeatureMatrix );

    //std::cout << "Descriptor: " << std::endl << FeatureMatrix << std::endl;

    /*for(size_t i = 0; i < sift_signatures->points.size(); i++)
    {
        for(size_t kk=0; kk < num_dimensions; kk++)
        {
            FeatureMatrix.at<float>(i,kk) = sift_signatures->points[i].histogram[kk];
        }
    }*/
    //std::cout << "FeatureVector: " << std::endl << FeatureVector << std::endl << std::endl;
}

void FeatureExtractor::computeSIFTFeatures(const PointInTPtr &inputX, const pcl::PointIndices &indices, cv::Mat &FeatureMatrix, size_t &num_dimensions)
{
    num_dimensions = 128;

    PointInTPtr sift_keypoints;
    sift_keypoints.reset(new pcl::PointCloud<PointT>());
    SIFTFeatureTPtr sift_signatures;
    sift_signatures.reset(new pcl::PointCloud<SIFTFeatureT>());
    std::vector<float> sift_scale;
    sift_estimator_->setIndices(indices);
    sift_estimator_->estimate(inputX, sift_keypoints, sift_signatures, sift_scale);
    FeatureMatrix = cv::Mat(sift_signatures->points.size(), num_dimensions, CV_32F);

    for(size_t i = 0; i < sift_signatures->points.size(); i++)
    {
        for(size_t kk=0; kk < num_dimensions; kk++)
        {
            FeatureMatrix.at<float>(i,kk) = sift_signatures->points[i].histogram[kk];
        }
    }
}

void FeatureExtractor::computeESFFeatures(const PointInTPtr &inputX, const pcl::PointIndices &indices, cv::Mat &FeatureMatrix, size_t &num_dimensions)
{
    /*esf_signatures_.reset(new pcl::PointCloud<ESFFeatureT>());
    esf_signatures_->width = models3D_->size();
    esf_signatures_->height = 1;
    esf_signatures_->resize(models3D_->size());*/

    num_dimensions = 640;

    pcl::PointCloud<ESFFeatureT>::CloudVectorType esf_signature;
    std::vector<Eigen::Vector3f> centroids;
    pcl::PointCloud<PointT>::Ptr pFilteredCloud, pProcessedCloud;
    pFilteredCloud.reset(new pcl::PointCloud<PointT>());
    pProcessedCloud.reset(new pcl::PointCloud<PointT>());
    pcl::copyPointCloud(*inputX, indices, *pFilteredCloud);
    esf_estimator_->estimate(pFilteredCloud, pProcessedCloud, esf_signature, centroids);
    FeatureMatrix = cv::Mat(1, num_dimensions, CV_32F);

    for(int kk = 0; kk < num_dimensions; kk++)
    {
       FeatureMatrix.at<float>(0,kk) = static_cast<double>(esf_signature[0].points[0].histogram[kk]);
    }
}

void FeatureExtractor::computeESFFeatures(const PointInTPtr &inputX, cv::Mat &FeatureMatrix, size_t &num_dimensions)
{
    /*esf_signatures_.reset(new pcl::PointCloud<ESFFeatureT>());
    esf_signatures_->width = models3D_->size();
    esf_signatures_->height = 1;
    esf_signatures_->resize(models3D_->size());*/

    num_dimensions = 640;

    pcl::PointCloud<ESFFeatureT>::CloudVectorType esf_signature;
    std::vector<Eigen::Vector3f> centroids;
    pcl::PointCloud<PointT>::Ptr pProcessedCloud;
    pProcessedCloud.reset(new pcl::PointCloud<PointT>());
    esf_estimator_->estimate(inputX, pProcessedCloud, esf_signature, centroids);
    FeatureMatrix = cv::Mat(1, num_dimensions, CV_32F);

    for(int kk = 0; kk < num_dimensions; kk++)
    {
        FeatureMatrix.at<float>(0,kk) = static_cast<double>(esf_signature[0].points[0].histogram[kk]);
   }
}

void FeatureExtractor::computeSHOTFeatures(const PointInTPtr &inputX, const pcl::PointIndices &indices, cv::Mat &FeatureMatrix, size_t &num_dimensions)
{
    num_dimensions = 352;

    boost::shared_ptr<faat_pcl::rec_3d_framework::UniformSamplingExtractor<PointT> >
            uniform_keypoint_extractor ( new faat_pcl::rec_3d_framework::UniformSamplingExtractor<PointT>);
    uniform_keypoint_extractor->setSamplingDensity (0.01f);
    uniform_keypoint_extractor->setFilterPlanar (true);

    boost::shared_ptr<faat_pcl::rec_3d_framework::KeypointExtractor<PointT> > keypoint_extractor;
    keypoint_extractor = boost::static_pointer_cast<faat_pcl::rec_3d_framework::KeypointExtractor<PointT> > (uniform_keypoint_extractor);

    boost::shared_ptr<faat_pcl::rec_3d_framework::PreProcessorAndNormalEstimator<PointT, pcl::Normal> > normal_estimator;
    normal_estimator.reset (new faat_pcl::rec_3d_framework::PreProcessorAndNormalEstimator<PointT, pcl::Normal>);
    normal_estimator->setCMR (false);
    normal_estimator->setDoVoxelGrid (true);
    normal_estimator->setRemoveOutliers (false);
    normal_estimator->setValuesForCMRFalse (0.003f, 0.02f);

    shot_estimator_->setNormalEstimator (normal_estimator);
    shot_estimator_->addKeypointExtractor (keypoint_extractor);
    shot_estimator_->setSupportRadius (0.04f);
    shot_estimator_->setAdaptativeMLS (false);

    pcl::PointCloud<PointT>::Ptr pProcessedCloud;
    pProcessedCloud.reset(new pcl::PointCloud<PointT>());
    PointInTPtr shot_keypoints;
    shot_keypoints.reset(new pcl::PointCloud<PointT>());

    SHOTFeatureTPtr shot_signatures;
    shot_signatures.reset(new pcl::PointCloud<SHOTFeatureT>());
    shot_estimator_->estimate(inputX, pProcessedCloud, shot_keypoints, shot_signatures);

    FeatureMatrix = cv::Mat(shot_signatures->points.size(), num_dimensions, CV_32F);

    for(size_t i = 0; i < shot_signatures->points.size(); i++)
    {
        for(size_t kk=0; kk < num_dimensions; kk++)
        {
            FeatureMatrix.at<float>(i,kk) = shot_signatures->points[i].histogram[kk];
        }
    }
    //boost::shared_ptr<faat_pcl::rec_3d_framework::LocalEstimator<PointT, pcl::Histogram<352> > > cast_estimator;
    //cast_estimator = boost::dynamic_pointer_cast<faat_pcl::rec_3d_framework::LocalEstimator<PointT, pcl::Histogram<352> > > (estimator);
}

void FeatureExtractor::computeCustomFeatures(const PointInTPtr &pInputX, const pcl::PointIndices &indices, cv::Mat &FeatureMatrix, size_t &num_dimensions)
{
    num_dimensions = 3;

   //Normalize N(0,1)

    pcl::PointCloud<PointT>::Ptr pFilteredCloud;
    pFilteredCloud.reset(new pcl::PointCloud<PointT>());
    pcl::copyPointCloud(*pInputX, indices, *pFilteredCloud);

    Eigen::Vector4f centroid;
    Eigen::Matrix3f covariance_matrix;
    pcl::computeMeanAndCovarianceMatrix(*pFilteredCloud, covariance_matrix, centroid);
    //Eigen::EigenSolver<Eigen::Matrix3f> es(covariance_matrix);
    //std::cout << "eigenvector and eigenvalues for cluster " << i << std::endl;
    //Eigen::Matrix3f eigenvec_matrix = es.eigenvectors().real();
    //std::cout << es.eigenvalues().real() << std::endl << " vec: " << es.eigenvectors().real() << std::endl;
    Eigen::Matrix3f eigvects;
    Eigen::Vector3f eigvals;
    pcl::eigen33(covariance_matrix, eigvects,  eigvals);
    //std:cout << "PCL Eigen: " << std::endl << eigvals << std::endl << eigvects << std::endl;

    Eigen::Vector3f centroid_transformed = eigvects.transpose() * centroid.topRows(3);

    Eigen::Matrix4f transformation_matrix = Eigen::Matrix4f::Zero(4,4);
    transformation_matrix.block<3,3>(0,0) = eigvects.transpose();
    transformation_matrix.block<3,1>(0,3) = -centroid_transformed;
    transformation_matrix(3,3) = 1;

    pcl::PointCloud<PointT>::Ptr pClusterPCl_transformed (new pcl::PointCloud<PointT>());
    pcl::transformPointCloud(*pFilteredCloud, *pClusterPCl_transformed, transformation_matrix);

    //pcl::transformPointCloud(*frame_, cluster_indices_int, *frame_eigencoordinates_, eigvects);
    PointT min_pt, max_pt;
    pcl::getMinMax3D(*pClusterPCl_transformed, min_pt, max_pt);
    std::cout << "Elongations along eigenvectors: " << max_pt.x - min_pt.x << ", " << max_pt.y - min_pt.y
              << ", " << max_pt.z - min_pt.z << std::endl;


    FeatureMatrix = cv::Mat(1, num_dimensions, CV_32F);
    FeatureMatrix.at<float>(0,0) = max_pt.x - min_pt.x;
    FeatureMatrix.at<float>(0,1) = max_pt.y - min_pt.y;
    FeatureMatrix.at<float>(0,2) = max_pt.z - min_pt.z;

    /*pcl::visualization::PCLVisualizer vis("cloud for segmentation");
    pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb_handler (pFilteredCloud);
    vis.addPointCloud<PointT>(pFilteredCloud, rgb_handler, "cloud");
    vis.spin();*/
}

void FeatureExtractor::computeKahanMeanAndStd(const std::vector<float> data_v, double &mean, double &std)
{
    // computes Mean and standard deviation for a vector with Kahan summation algorithm
    double sum = 0, sum2=0, c = 0, varSum=0, var_c = 0, varSum2;

    for (size_t i=0; i < data_v.size(); i++)
    {
        double y = data_v[i] - c;
        double t = sum + y;
        c = (t - sum) - y;
        sum = t;
        sum2 += data_v[i];
    }

    mean = sum / data_v.size();
    double mean2 = sum2 / data_v.size();  // just for comparison

    if(data_v.size()>1)
    {
        for (size_t i=0; i < data_v.size(); i++)
        {
            double data = (data_v[i] - mean) * (data_v[i] - mean);
            double y = data - var_c;
            double t = varSum + y;
            var_c = (t - varSum) - y;
            varSum = t;
            varSum2 += data;
        }

        std = sqrt(varSum / (data_v.size() - 1) );
        double std2 = sqrt(varSum2 / (data_v.size() - 1) );  // just for comparison

    }
    else
        std = 0;
}

void FeatureExtractor::computeCustomFeatures(const cv::Mat &inputX, cv::Mat &FeatureMatrix, size_t &num_dimensions)
{
   //Normalize N(0,1)
    num_dimensions = 6;
    FeatureMatrix = cv::Mat::zeros(1, num_dimensions, CV_32F);

    if(inputX.channels() < 3)
    {
        std::cerr << "Input Matrix has less than 3 channels. Cannot access RGB values." << std::endl;
    }
    else
    {
        cv::Mat inputXlab;
        cv::cvtColor(inputX, inputXlab, cv::COLOR_RGB2Lab);

        std::vector<float> L_v, A_v, B_v;

        // Compute mean values for LAB space with Kahan summation algorithm


        size_t points=0;

        cv::Mat_<cv::Vec3d> lab;
        MyColorConverter::convertBGRtoLAB(inputX, lab);

        for(size_t row_id=0; row_id < inputX.rows; row_id++)
        {
            for(size_t col_id=0; col_id < inputX.cols; col_id++)
            {
                float L, A, B2;
                unsigned char B = inputX.at<cv::Vec3b>(row_id, col_id)[0];
                unsigned char G = inputX.at<cv::Vec3b>(row_id, col_id)[1];
                unsigned char R = inputX.at<cv::Vec3b>(row_id, col_id)[2];
                MyColorConverter::RGB2CIELAB (R, G, B, L, A, B2);

                L_v.push_back(L);
                A_v.push_back(A);
                B_v.push_back(B2);

                /*std::cout << "(" << row_id << "," << col_id << ")"
                          <<  " B: " << B
                          << "; G: " << G
                          << "; R: " << R
                          << std::endl;

                std::cout << "(" << row_id << "," << col_id << ")"
                          <<  " L: " << static_cast<int>(inputXlab.at<cv::Vec3b>(row_id, col_id)[0])
                          << "; A: " << static_cast<int>(inputXlab.at<cv::Vec3b>(row_id, col_id)[1])
                          << "; B: " << static_cast<int>(inputXlab.at<cv::Vec3b>(row_id, col_id)[2])
                          << std::endl;


                std::cout << "(" << row_id << "," << col_id << ")"
                          << " L_: " << L
                          << "; A_: " << A
                          << "; B_: " << B2
                          << std::endl;

                std::cout << "(" << row_id << "," << col_id << ")"
                          <<  " LL_: " << lab.at<cv::Vec3d>(row_id, col_id)[0]
                          << "; AA_: " << lab.at<cv::Vec3d>(row_id, col_id)[1]
                          << "; BB_: " << lab.at<cv::Vec3d>(row_id, col_id)[2]
                          << std::endl;*/
            }
        }

        /*double Lmean2 = cv::mean(inputXlab)[0];
        double Amean2 = cv::mean(inputXlab)[1];
        double Bmean2 = cv::mean(inputXlab)[2];*/

        double L_mean, L_stdDev, A_mean, A_stdDev, B_mean, B_stdDev;
        computeKahanMeanAndStd(L_v, L_mean, L_stdDev);
        computeKahanMeanAndStd(A_v, A_mean, A_stdDev);
        computeKahanMeanAndStd(B_v, B_mean, B_stdDev);

        /*cv::Scalar mean, stdDev;
        cv::meanStdDev(inputXlab, mean, stdDev);

        double Lmean = mean[0];
        double Amean = mean[1];
        double Bmean = mean[2];

        double LstdDev = stdDev[0];
        double AstdDev = stdDev[1];
        double BstdDev = stdDev[2];*/

        FeatureMatrix.at<float>(0,0) = L_mean;
        FeatureMatrix.at<float>(0,1) = A_mean;
        FeatureMatrix.at<float>(0,2) = B_mean;
        FeatureMatrix.at<float>(0,3) = L_stdDev;
        FeatureMatrix.at<float>(0,4) = A_stdDev;
        FeatureMatrix.at<float>(0,5) = B_stdDev;

    }

}
