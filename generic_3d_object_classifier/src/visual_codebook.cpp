#include "visual_codebook.h"

void VisualCodebook::readCodebookFromFile(const std::string base_dir)
{
    std::stringstream filename;
    filename << base_dir << "/codebook.yml";
    cv::FileStorage codebookFile(filename.str(), cv::FileStorage::READ);
    codebookFile["DescriptorClusterCenters"] >> desc_cluster_centers_;
    codebookFile.release();
}

void VisualCodebook::writeCodebookToFile(const std::string base_dir)
{
    boost::filesystem::path dir(base_dir);

    if(!(boost::filesystem::exists(dir)))
    {
         std::cout << base_dir << " does not exist..." << std::endl;

         if (boost::filesystem::create_directory(dir))
             std::cout << base_dir << " successfully created !" << std::endl;
     }

    std::stringstream filename;
    filename << base_dir << "/codebook.yml";

    cv::FileStorage codebookFile(filename.str(), cv::FileStorage::WRITE);
    codebookFile << "DescriptorClusterCenters" << desc_cluster_centers_;
    codebookFile.release();
}

void VisualCodebook::computeHist(const cv::Mat labels, std::vector<double> & hist_v)
{
    std::vector<double> hist_tmp_v(num_clusters_);

    for(size_t hist_bin_id=0; hist_bin_id < num_clusters_; hist_bin_id++)
    {
        hist_tmp_v[hist_bin_id]=0;
    }

    for (size_t signature_id=0; signature_id < labels.rows; signature_id++)
    {
        int hist_bin_id = labels.at<int>(signature_id, 0);
        hist_tmp_v[hist_bin_id] += 1;
    }

    for(size_t hist_bin_id=0; hist_bin_id < num_clusters_; hist_bin_id++)
    {   //normalize such that sum(hist)=1
        hist_tmp_v[hist_bin_id] = hist_tmp_v[hist_bin_id] / labels.rows;
                //hist_weights_[hist_bin_id] * (hist_tmp_v[hist_bin_id] / labels.rows);
    }

    // discard stop words----------------
    for(size_t hist_bin_id=0; hist_bin_id < num_clusters_; hist_bin_id++)
    {
        if(!hist_bin_is_discarded_[hist_bin_id])
        {
            hist_v.push_back(hist_tmp_v[hist_bin_id]);
        }
    }
}

void VisualCodebook::computeHists(
        const std::vector<std::vector<cv::Mat > > labels_per_class_per_view,
        std::vector<std::vector<std::vector<double> > > & hist_per_class_per_view_v)
{
    int num_classes = labels_per_class_per_view.size();

    std::vector<std::vector<std::vector<double> > > hist_tmp_v(num_classes);
    hist_per_class_per_view_v.resize(num_classes);

    for(size_t class_id=0; class_id < num_classes; class_id++)
    {
        int num_views_in_class = labels_per_class_per_view[class_id].size();
        hist_per_class_per_view_v[class_id].resize(num_views_in_class);

        for (size_t  view_id=0; view_id < num_views_in_class; view_id++)
        {
            computeHist(labels_per_class_per_view[class_id][view_id], hist_per_class_per_view_v[class_id][view_id]);
        }
    }
}

void VisualCodebook::computeCodebook(const std::vector<std::vector<cv::Mat> >
                                        &signatures_per_class_per_view_v,
                                        const size_t num_dimensions,
                                        std::vector<std::vector<cv::Mat> >&labels)
{
    //count number of signatures

    size_t num_signatures=0;
    size_t total_num_views=0;

    for(size_t class_id=0; class_id < signatures_per_class_per_view_v.size(); class_id++)
    {
        for (size_t  view_id=0; view_id < signatures_per_class_per_view_v[class_id].size(); view_id++)
        {
                num_signatures += signatures_per_class_per_view_v[class_id][view_id].rows;
        }
    }

    cv::Mat signatures_concatenated;
    size_t conc_row_id = 0;

    for(size_t class_id=0; class_id < signatures_per_class_per_view_v.size(); class_id++)
    {
        for (size_t  view_id=0; view_id < signatures_per_class_per_view_v[class_id].size(); view_id++)
        {
                if(signatures_concatenated.rows==0)
                {
                    signatures_concatenated = signatures_per_class_per_view_v[class_id][view_id];
                }
                else
                {
                    if(signatures_per_class_per_view_v[class_id][view_id].rows > 0)
                    {
                        cv::vconcat(signatures_concatenated, signatures_per_class_per_view_v[class_id][view_id], signatures_concatenated);
                    }
                }
                total_num_views++;
                //signatures_per_class_per_view_v[class_id][view_id].row(signature_id).copyTo(signatures_concatenated.row(conc_row_id));
                //conc_row_id ++;
        }
    }

    //std::cout << "signatures concatenated: " << std::endl << signatures_concatenated << std::endl;

    //desc_cluster_centers_ = cv::Mat(num_clusters_, num_dimensions, CV_32FC1);
    cv::Mat labels_concatenated;
    cv::kmeans(signatures_concatenated, num_clusters_, labels_concatenated, cv::TermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 100, 1.0), 1, cv::KMEANS_RANDOM_CENTERS, desc_cluster_centers_);

    std::cout << "labels = "<< std::endl << " "  << labels_concatenated << std::endl << std::endl << std::endl;
    std::cout << "centers = "<< std::endl << " "  << desc_cluster_centers_ << std::endl << std::endl << std::endl;

    tree_.reset(new cv::flann::Index (desc_cluster_centers_, cv::flann::KDTreeIndexParams(4)));

    computeHistWeights(labels_concatenated, total_num_views);
    // Writing labels into corresponding view/class
    conc_row_id = 0;
    labels.resize(signatures_per_class_per_view_v.size());
    hist_frequencies_per_class_.resize(signatures_per_class_per_view_v.size());

    for(size_t class_id=0; class_id < signatures_per_class_per_view_v.size(); class_id++)
    {
        labels[class_id].resize(signatures_per_class_per_view_v[class_id].size());
        hist_frequencies_per_class_[class_id].resize(num_clusters_);
        hist_frequencies_.resize(num_clusters_);
        for(size_t hist_id = 0; hist_id < num_clusters_; hist_id++)
        {
            hist_frequencies_per_class_[class_id][hist_id]=0;
            hist_frequencies_[hist_id]=0;
        }

        for (size_t  view_id=0; view_id < signatures_per_class_per_view_v[class_id].size(); view_id++)
        {
            labels[class_id][view_id] = cv::Mat(signatures_per_class_per_view_v[class_id][view_id].rows, 1, CV_32S);

            for(size_t signature_id = 0; signature_id < signatures_per_class_per_view_v[class_id][view_id].rows; signature_id++)
            {
                labels[class_id][view_id].at<int>(signature_id, 0) = labels_concatenated.at<int>(conc_row_id, 0);
                hist_frequencies_per_class_[class_id][labels[class_id][view_id].at<int>(signature_id, 0)] ++;
                hist_frequencies_[labels[class_id][view_id].at<int>(signature_id, 0)] ++;
                conc_row_id++;
            }
        }
    }
    computeStopList();
    int debug=1;
}

void VisualCodebook::computeStopList()
{
    double top_stop_percent = 0.00;//0.05;
    double bottom_stop_percent = 0.00;//0.10;

    size_t num_top_stop_percent = static_cast<size_t>(num_clusters_ * top_stop_percent);
    size_t num_bottom_stop_percent = static_cast<size_t>(num_clusters_ * bottom_stop_percent);

    hist_bin_is_discarded_.resize(num_clusters_);
    for(size_t hist_id=0; hist_id < num_clusters_; hist_id++)
    {
        hist_bin_is_discarded_[hist_id] = 0;
    }

    for(size_t i=0; i < num_top_stop_percent; i++)
    {
        size_t max_value = 0;
        size_t max_id = 0;

        for(size_t hist_id=0; hist_id < num_clusters_; hist_id++)
        {
            if( ! hist_bin_is_discarded_[hist_id] )
            {
                if( hist_frequencies_[hist_id] > max_value)
                {
                    max_value = hist_frequencies_[hist_id];
                    max_id = hist_id;
                }
            }
        }
        hist_bin_is_discarded_[max_id] = 1;
    }


    for(size_t i=0; i < num_bottom_stop_percent; i++)
    {
        size_t min_value = std::numeric_limits<std::size_t>::max();
        size_t min_id = 0;

        for(size_t hist_id=0; hist_id < hist_frequencies_.size(); hist_id++)
        {
            if( ! hist_bin_is_discarded_[hist_id] )
            {
                if( hist_frequencies_[hist_id] < min_value)
                {
                    min_value = hist_frequencies_[hist_id];
                    min_id = hist_id;
                }
            }
        }
        hist_bin_is_discarded_[min_id] = 1;
    }

}

void VisualCodebook::computeLabels(const cv::Mat &signatures, cv::Mat &labels)
{
    cv::Mat dists;
    tree_->knnSearch(signatures, labels, dists, 1, cv::flann::SearchParams());
    //std::cout << "indices" << std::endl << labels_per_class_per_view_v[class_id][view_id] << std::endl;
    //std::cout << "dists" << std::endl << dists << std::endl;
}

void VisualCodebook::computeLabelsVectors(const std::vector<std::vector<cv::Mat> >
                                      &signatures_per_class_per_view_v,
                                      std::vector<std::vector<cv::Mat> >&labels_per_class_per_view_v)
{
    size_t num_classes = signatures_per_class_per_view_v.size();
    labels_per_class_per_view_v.resize(num_classes);

    for(size_t class_id=0; class_id < num_classes; class_id++)
    {
        int num_views_in_class = signatures_per_class_per_view_v[class_id].size();
        labels_per_class_per_view_v[class_id].resize(num_views_in_class);

        for (size_t  view_id=0; view_id < num_views_in_class; view_id++)
        {
            computeLabels(signatures_per_class_per_view_v[class_id][view_id], labels_per_class_per_view_v[class_id][view_id]);
        }
    }
}

void VisualCodebook::computeHistWeights(cv::Mat labels_concatenated, size_t total_num_views)
{
    // according to paper Video Google: A Text Retrieval Approach to Object Matching in Video (Sivic and Zisserman)

    hist_weights_.resize(num_clusters_);

    for(size_t hist_bin_id=0; hist_bin_id < num_clusters_; hist_bin_id++)
    {
        hist_weights_[hist_bin_id] = 0;
    }

    for (size_t  label_id=0; label_id < labels_concatenated.rows; label_id++)
    {
        hist_weights_[labels_concatenated.at<int>(label_id,0)] ++;
    }

    for(size_t hist_bin_id=0; hist_bin_id < num_clusters_; hist_bin_id++)
    {
        hist_weights_[hist_bin_id] = log( total_num_views / hist_weights_[hist_bin_id] );
    }
}
