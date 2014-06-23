#ifndef VISUAL_CODEBOOK_H
#define VISUAL_CODEBOOK_H

#include <iostream>
#include <vector>
#include <fstream>
#include <string>
#include <math.h>
#include <limits>

#include <pcl/common/common.h>
#include <cv.h>
#include <boost/filesystem.hpp>

class VisualCodebook{
private:
    cv::Mat desc_cluster_centers_;
    size_t num_clusters_, num_stops_;
    std::vector<double> hist_weights_;
    std::vector<std::vector<double> > hist_frequencies_per_class_;
    std::vector<double> hist_frequencies_;
    std::vector<double> hist_bin_is_discarded_;
    boost::shared_ptr< cv::flann::Index> tree_;

public:
    VisualCodebook()
    {
        num_clusters_ = 100;
    }

    void computeHists(
            const std::vector<std::vector< cv::Mat > > labels_per_class_per_view_v,
            std::vector<std::vector<std::vector<double> > > & hist_per_class_per_view_v
            );

    void computeHist(
            const cv::Mat labels,
            std::vector<double> & hist_v);

    void computeCodebook(
            const std::vector<std::vector<cv::Mat> > &signatures_per_class_per_view_v,
            const size_t num_dimensions,
            std::vector<std::vector<cv::Mat> >&labels
            );

    void computeLabelsVectors(
            const std::vector<std::vector<cv::Mat> > &signatures_per_class_per_view_v,
            std::vector<std::vector<cv::Mat> >&labels_per_class_per_view_v
            );

    void computeLabels(const cv::Mat &signatures, cv::Mat &labels);


    void writeCodebookToFile(const std::string base_dir);

    void readCodebookFromFile(const std::string base_dir);

    void getDescClusterCenters(cv::Mat &desc_cluster_centers)
    {
        desc_cluster_centers =  desc_cluster_centers_;
    }

    size_t getNumClusters()
    {
        return num_clusters_;
    }

    void setNumClusters(size_t num_clusters)
    {
        num_clusters_ = num_clusters;
    }

    void computeHistWeights(cv::Mat labels_concatenated, size_t total_num_views);

    void computeStopList();

};

#endif //VISUAL_CODEBOOK_H
