#include "my_classifier.h"
#include <iostream>
#include <fstream>
#include <string>
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */
#include <algorithm>
#include <iterator>
#include <math.h>

bool is_number(const std::string& s)
{
    std::string::const_iterator it = s.begin();
    while (it != s.end() && std::isdigit(*it)) ++it;
    return !s.empty() && it == s.end();
}

void MyClassifier::displayComputedFeatures()
{
    std::cout << "Activated Features for 3D: " << std::endl;
    if((features3d_ & SIFT_3D) == SIFT_3D)
    {
        std::cout << " SIFT" << std::endl;
    }

    if((features3d_ & ESF_3D) == ESF_3D)
    {
        std::cout << " ESF" << std::endl;
    }

    if((features3d_ & SHOT_3D) == SHOT_3D)
    {
        std::cout << " SHOT" << std::endl;
    }

    if((features3d_ & CUSTOM_3D) == CUSTOM_3D)
    {
        std::cout << " CUSTOM" << std::endl;
    }

    std::cout << std::endl << std::endl;

    std::cout << "Activated Features for 2D: " << std::endl;
    if((features2d_ & SIFT_2D) == SIFT_2D)
    {
        std::cout << " SIFT" << std::endl;
    }

    if((features2d_ & CUSTOM_2D) == CUSTOM_2D)
    {
        std::cout << " CUSTOM" << std::endl;
    }
}

void MyClassifier::writeGlobalSignaturesToFile(
        const std::string base_dir,
        const std::vector<std::vector<std::vector<double> > > & hist_per_class_per_view_v)
{
    boost::filesystem::path dir(base_dir);

    if(!(boost::filesystem::exists(dir)))
    {
         std::cout << base_dir << " does not exist..." << std::endl;

         if (boost::filesystem::create_directory(dir))
             std::cout << base_dir << " successfully created !" << std::endl;
     }

    for(size_t class_id=0; class_id < hist_per_class_per_view_v.size(); class_id++)
    {
        std::ofstream myfile;
        std::stringstream filename;
        filename << base_dir << "/global_signatures_class_" << class_id << ".txt";
        myfile.open(filename.str().c_str());

        if(!myfile)
        {
            std::cerr << "Could not write to " << filename.str() << std::endl;
        }
        else
        {
            for(size_t view_id=0; view_id < hist_per_class_per_view_v[class_id].size(); view_id++)
            {
                size_t num_clusters = hist_per_class_per_view_v[class_id][view_id].size();
                for(size_t hist_bin_id=0; hist_bin_id < num_clusters; hist_bin_id++)
                {
                    myfile << hist_per_class_per_view_v[class_id][view_id][hist_bin_id] << " " ;
                }
                myfile << "\n";
            }
           myfile.close();
        }
     }
}

void MyClassifier::init()
{
    std::string indices_prefix = "object_indices_";

    bf::path trained_dir_bf = trained_dir_;
    if(bf::is_directory(trained_dir_))
    {
        std::vector<std::string> training_filenames;

        faat_pcl::utils::getFilesInDirectory(trained_dir_bf, training_filenames, "", ".*.txt", false);
        for(int i=0; i<training_filenames.size(); i++)
        {
            std::cout << training_filenames[i] << std::endl;
        }
        num_classes_ = training_filenames.size();

        if(training_filenames.empty())
        {
            std::cerr << "There are no training files in " << trained_dir_ << ". Re-Training..." << std::endl;
            force_retrain_ = true;
        }
    }
    else
    {
        std::cerr << "Training directory(" << trained_dir_ << ") does not exist. Re-Training..." << std::endl;
        force_retrain_ = true;
        if (boost::filesystem::create_directory(trained_dir_bf))
        {
            std::cout << "Successfully created trained dir. " << std::endl;
        }
        else
        {
            std::cerr << "Could not create trained dir. Writing to localhome directory ~. " << std::endl;
            trained_dir_ = "~";
        }
    }

    // load a .pcd file, extract indices of object and classify
    if(bf::exists(test_filename_))
    {
        std::string directory, filename;
        char sep = '/';
#ifdef _WIN32
        sep = '\\';
#endif

        size_t position = test_filename_.rfind(sep);
        if (position != std::string::npos)
        {
            directory = test_filename_.substr(0, position);
            filename = test_filename_.substr(position+1, test_filename_.length()-1);
        }

        std::stringstream path_oi;
        path_oi << directory << "/" << indices_prefix << filename ;

        if(bf::exists(path_oi.str()))
        {
            pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
            pcl::io::loadPCDFile (test_filename_, *cloud);

            pcl::PointCloud<IndexPoint> obj_indices_cloud;
            pcl::io::loadPCDFile (path_oi.str(), obj_indices_cloud);
            pcl::PointCloud<PointT>::Ptr pFilteredCloud;
            pFilteredCloud.reset(new pcl::PointCloud<PointT>());
            pcl::PointIndices indices;
            indices.indices.resize(obj_indices_cloud.points.size());
            for(size_t kk=0; kk < obj_indices_cloud.points.size(); kk++)
                indices.indices[kk] = obj_indices_cloud.points[kk].idx;
            pcl::copyPointCloud(*cloud, indices, *pFilteredCloud);

            this->setInputCloud(*pFilteredCloud);
            std::cout << "Test point cloud and indices loaded for file: " << test_filename_ << std::endl;
        }
    }
}

void MyClassifier::setInputCloud(const pcl::PointCloud<PointT> &cloud)
{
    *pInputCloud_ = cloud;
}

void MyClassifier::writeSignaturesToFile(
        const std::string base_dir,
        const std::vector<cv::Mat> & signatures_per_view_v,
        size_t class_id,
        size_t num_dimensions)
{
    boost::filesystem::path dir(base_dir);

    if(!(boost::filesystem::exists(dir)))
    {
         std::cout << base_dir << " does not exist..." << std::endl;

         if (boost::filesystem::create_directory(dir))
             std::cout << base_dir << " successfully created !" << std::endl;
     }

    std::ofstream myfile;
    std::stringstream filename;
    filename << base_dir << "/signatures_" << class_id << "_dim_" << num_dimensions << ".txt";
    myfile.open(filename.str().c_str());

    if(!myfile)
    {
        std::cerr << "Could not write to " << filename.str() << std::endl;
    }
    else
    {
        for(size_t view_id=0; view_id < signatures_per_view_v.size(); view_id++)
        {
            for(size_t signature_id=0; signature_id < signatures_per_view_v[view_id].rows; signature_id++)
            {
                for(size_t feature_attribute_id=0; feature_attribute_id<signatures_per_view_v[view_id].cols; feature_attribute_id++)
                {
                    myfile << signatures_per_view_v[view_id].at<double>(signature_id, feature_attribute_id) << " " ;
                }
            }
            myfile << "\n";
        }
       myfile.close();
    }
}

std::vector<std::vector<double> > MyClassifier::readMatrixFromFile(
        const std::string filename, const std::string delimiter)
{
    std::vector<std::vector<double> > data;

    std::ifstream file;
    file.open(filename.c_str());

    if(!file.is_open())
    {
        std::cerr << "Failed to open file " << filename << std::endl;
        return data;
    }
    std::string line;
    size_t line_id = 0;
    while( std::getline(file,line))
    {
        std::vector<double> data_tmp;

        size_t pos = 0;
        std::string token;
        while ((pos = line.find(delimiter)) != std::string::npos) {
            token = line.substr(0, pos);
            line.erase(0, pos + delimiter.length());
            data_tmp.push_back(atof(token.c_str()));
        }
        // check if there is another number at the end (depends if blank space at end of line or not)
        if (is_number(line))
        {
            data_tmp.push_back(atof(line.c_str()));
        }
        data.push_back(data_tmp);
    }
}

template <typename MTPtr>
void MyClassifier::assignClassToId(const boost::shared_ptr<const std::vector<MTPtr> > models)
{
    for(size_t i=0; i < models->size(); i++)
    {
        if(class_map_.find(models->at(i)->class_) == class_map_.end())
        {
            size_t class_label = class_map_.size();
            std::cout << "Class " << models->at(i)->class_
                      << " corresponds to id " << class_label << std::endl;
            class_map_[models->at(i)->class_] = class_label;
        }
    }
}


void MyClassifier::mergeGlobalDescriptors(
        const std::vector< std::vector< std::vector< std::vector< double > > > > &global_signatures_per_featureType_per_class_per_view_v,
        std::vector< std::vector< std::vector< double > > > &merged_global_signatures_per_class_per_view_v)
{
    size_t featTypes = global_signatures_per_featureType_per_class_per_view_v.size();

    for(size_t featTyp=0; featTyp < featTypes; featTyp++)
    {
        size_t num_classes = global_signatures_per_featureType_per_class_per_view_v[featTyp].size();
        if(merged_global_signatures_per_class_per_view_v.size() == 0)
        {
            merged_global_signatures_per_class_per_view_v.resize(num_classes);
        }

        for(size_t class_id=0; class_id < num_classes; class_id++)
        {
            size_t num_views = global_signatures_per_featureType_per_class_per_view_v[featTyp][class_id].size();
            if( merged_global_signatures_per_class_per_view_v[class_id].size() == 0)
            {
                merged_global_signatures_per_class_per_view_v[class_id].resize(num_views);
            }

            for(size_t view_id=0; view_id < num_views; view_id++)
            {
                size_t num_dimensions = global_signatures_per_featureType_per_class_per_view_v[featTyp][class_id][view_id].size();
                for(size_t feature_Id=0; feature_Id<num_dimensions; feature_Id++)
                {
                    double tmp = global_signatures_per_featureType_per_class_per_view_v[featTyp][class_id][view_id][feature_Id];
                    merged_global_signatures_per_class_per_view_v[class_id][view_id].push_back(tmp);
                }
            }
        }
    }

}

void MyClassifier::trainClassifier()
{
    displayComputedFeatures();
    pImgSource_->setPath(models_dir_);
    pImgSource_->generate();
    models2D_ = pImgSource_->getModels();

    pPCSource_->setPath(models_dir_);
    std::string dummy = "";
    pPCSource_->generate(dummy);
    cast_source_ = boost::static_pointer_cast<faat_pcl::rec_3d_framework::UnregisteredViewsSource<PointT> > (pPCSource_);
    models3D_ = cast_source_->getModels ();

    // ---Assign class names to unique discrete identifiers ------
    assignClassToId<ModelTPtr>(models3D_);
    assignClassToId<Model2DTPtr>(models2D_);
    num_classes_ = class_map_.size();

    if ( force_retrain_ )
    {
        std::vector< std::vector< cv::Mat > >      local_signatures_per_class_per_view_v;
        std::vector< std::vector< std::vector< double > > > global_signatures_per_class_per_view_GLOBAL_v;
        std::vector< std::vector< std::vector< std::vector< double > > > > global_signatures_per_class_per_view_v;
        std::vector< std::vector< std::string > > view_name_per_class_v;
        std::vector< std::vector< std::vector< double > > > global_signatures_merged;

        local_signatures_per_class_per_view_v.resize( num_classes_ );
        global_signatures_per_class_per_view_GLOBAL_v.resize( num_classes_ );
        view_name_per_class_v.resize( num_classes_);

        size_t num_dimensions_global;
        size_t num_dimensions_local;

        for(size_t i=0; i < models3D_->size(); i++)
        {
            cv::Mat FeatureMatrix;
            std::vector< double > glob_signature_temp;
            std::string class_name = models3D_->at(i)->class_;
            std::string view_name = models3D_->at(i)->id_;
            size_t class_id = class_map_[ class_name ];

            view_name_per_class_v[class_id].push_back( view_name );

            if((features3d_ & SIFT_3D) == SIFT_3D)
            {
                feature_extractor_->computeSIFTFeatures(models3D_->at(i)->views_->at(0), models3D_->at(i)->indices_->at(0), FeatureMatrix, num_dimensions_local);
                local_signatures_per_class_per_view_v[class_id].push_back(FeatureMatrix);
            }

            if((features3d_ & ESF_3D) == ESF_3D)
            {
                feature_extractor_->computeESFFeatures(models3D_->at(i)->views_->at(0), models3D_->at(i)->indices_->at(0), FeatureMatrix, num_dimensions_global);
                for(size_t feat_id = 0; feat_id < FeatureMatrix.cols; feat_id ++)
                {
                    glob_signature_temp.push_back(FeatureMatrix.at<float>(0,feat_id));
                }
            }

            if((features3d_ & SHOT_3D) == SHOT_3D)
            {
                //feature_extractor_->computeSHOTFeatures(models3D_->at(i)->views_->at(0), models3D_->at(i)->indices_->at(0), FeatureMatrix, num_dimensions_local);
                // :::::
                // :::::
                // :::::
            }

            if((features3d_ & CUSTOM_3D) == CUSTOM_3D)
            {
                feature_extractor_->computeCustomFeatures(models3D_->at(i)->views_->at(0), models3D_->at(i)->indices_->at(0), FeatureMatrix, num_dimensions_global);
                for(size_t feat_id = 0; feat_id < FeatureMatrix.cols; feat_id ++)
                {
                    glob_signature_temp.push_back(FeatureMatrix.at<float>(0,feat_id));
                }
            }
            global_signatures_per_class_per_view_GLOBAL_v[class_id].push_back(glob_signature_temp);
        }

        for(size_t i=0; i < models2D_->size(); i++)
        {
            cv::Mat FeatureMatrix;
            std::vector<double> glob_signature_temp;
            std::string class_name = models2D_->at(i)->class_;
            std::string view_name = models2D_->at(i)->id_;
            size_t class_id = class_map_[class_name];

            view_name_per_class_v[class_id].push_back(view_name);

            if((features2d_ & SIFT_2D) == SIFT_2D)
            {
                std::cout << "computing SIFT for " << view_name << std::endl;
                feature_extractor_->computeSIFTFeatures(*(models2D_->at(i)->view_), FeatureMatrix, num_dimensions_local);
                local_signatures_per_class_per_view_v[class_id].push_back(FeatureMatrix);
            }

            if((features2d_ & CUSTOM_2D) == CUSTOM_2D)
            {
                feature_extractor_->computeCustomFeatures(*(models2D_->at(i)->view_), FeatureMatrix, num_dimensions_global);
                for(size_t feat_id = 0; feat_id < FeatureMatrix.cols; feat_id ++)
                {
                    glob_signature_temp.push_back(FeatureMatrix.at<float>(0,feat_id));
                }
            }

            global_signatures_per_class_per_view_GLOBAL_v[class_id].push_back(glob_signature_temp);
        }

        global_signatures_per_class_per_view_v.push_back(global_signatures_per_class_per_view_GLOBAL_v);


        //--represent-local-descriptors-as-global-descriptors-(clustering-by-k-means-algorithm)-----------
        if( ((features2d_ & SIFT_2D) == SIFT_2D) || ((features3d_ & SIFT_3D) == SIFT_3D) )
        {
            std::map<std::string, size_t>::iterator class_it;
            for(class_it = class_map_.begin(); class_it!=class_map_.end(); class_it++)
            {
                writeSignaturesToFile( "/home/thomas/data/local_signatures",
                                       local_signatures_per_class_per_view_v[class_it->second],
                                       class_it->second,
                                       num_dimensions_local );
                /*writeSignaturesToFile( "/home/thomas/data/local_signatures",
                                       global_signatures_per_class_per_view_v[class_it->second],
                                       class_it->second,
                                       num_dimensions_global );*/
            }

            std::vector< std::vector< std::vector< double > > > global_signatures_per_class_per_view_SIFT_v;
            global_signatures_per_class_per_view_SIFT_v.resize(num_classes_);

            cv::Mat descClusterCenters;
            std::vector< std::vector< cv::Mat > > labels_per_class_per_view_v_dummy;
            std::vector< std::vector< cv::Mat > > labels_per_class_per_view_v;

            siftCodebook_->setNumClusters(80);
            siftCodebook_->computeCodebook(local_signatures_per_class_per_view_v, num_dimensions_local, labels_per_class_per_view_v_dummy);
            siftCodebook_->getDescClusterCenters(descClusterCenters);
            siftCodebook_->computeLabelsVectors(local_signatures_per_class_per_view_v, labels_per_class_per_view_v);
            siftCodebook_->computeHists(labels_per_class_per_view_v, global_signatures_per_class_per_view_SIFT_v);
            siftCodebook_->writeCodebookToFile(trained_dir_);

            global_signatures_per_class_per_view_v.push_back(global_signatures_per_class_per_view_SIFT_v);
        }

        mergeGlobalDescriptors(global_signatures_per_class_per_view_v, global_signatures_merged);
        writeGlobalSignaturesToFile(trained_dir_, global_signatures_merged);
        //writeGlobalSignaturesToFile(trained_dir_, global_signatures_per_class_per_view_GLOBAL_v);
    }
    else
    {
        siftCodebook_->readCodebookFromFile(trained_dir_);
    }
    std::vector<std::vector<double> > data_train;
    std::vector<double> target_train;

    std::map<std::string, size_t>::const_iterator class_it;
    for (class_it = class_map_.begin(); class_it != class_map_.end(); class_it ++)
    {
        std::stringstream filename_class_hist;
        filename_class_hist << trained_dir_ << "/global_signatures_class_" << class_it->second << ".txt";

        std::vector<std::vector<double> > data_train_class = readMatrixFromFile(filename_class_hist.str(), " ");

        for(size_t i=0; i<data_train_class.size(); i++)
        {
            target_train.push_back(class_it->second);
            data_train.push_back(data_train_class[i]);
        }
    }
    pSVM->initSVM();
    pSVM->setNumClasses(num_classes_);
    pSVM->shuffleTrainingData(data_train, target_train);
    pSVM->dokFoldCrossValidation(data_train, target_train, 5, exp2(-5), exp2(15), 2, exp2(-15), exp2(4), 2);
    pSVM->dokFoldCrossValidation(data_train, target_train, 5, pSVM->svm_para_.C / 2, pSVM->svm_para_.C * 2, 1.2, pSVM->svm_para_.gamma / 2, pSVM->svm_para_.gamma * 2, 1.2);
    pSVM->sortTrainingData(data_train, target_train);
    pSVM->computeSvmModel(data_train, target_train);
}

void MyClassifier::classify()
{
    size_t num_dimensions;
    cv::Mat image = cv::imread("/home/thomas/data/Cat50_TestDB_small/image_color/1.png", CV_LOAD_IMAGE_COLOR);
    cv::Mat sift_signatures_mat;
    feature_extractor_->computeSIFTFeatures(image, sift_signatures_mat, num_dimensions);

    size_t num_signatures = sift_signatures_mat.rows;
    cv::Mat label_test(num_signatures, 1, CV_32S);
    cv::Mat descClusterCenters;
    siftCodebook_->getDescClusterCenters(descClusterCenters);

    for(size_t i=0; i < num_signatures; i++)
    {
        label_test.at<int>(i,0) = 0;
        double shortest_dist= DBL_MAX;
        for(size_t kk=0; kk<descClusterCenters.rows; kk++)
        {
            std::cout << descClusterCenters.row(kk).size() <<
                         descClusterCenters.row(kk).type() <<
                         sift_signatures_mat.row(i).size() <<
                         sift_signatures_mat.row(i).type() << std::endl;
            double dist = cv::norm(descClusterCenters.row(kk), sift_signatures_mat.row(i), cv::NORM_L2);
            if(dist<shortest_dist)
            {
                shortest_dist = dist;
                label_test.at<int>(i,0) = kk;
            }
        }
    }
    std::cout << "labels = "<< std::endl << " "  << label_test << std::endl << std::endl;

    size_t num_clusters = siftCodebook_->getNumClusters();
    std::vector<double> hist_train(num_clusters);
    for(size_t hist_bin_id=0; hist_bin_id < num_clusters; hist_bin_id++)
    {
        hist_train [hist_bin_id]=0;
    }

    for (size_t row_id=0; row_id < label_test.rows; row_id++)
    {
        int hist_bin_id = label_test.at<int>(row_id, 0);
        hist_train [hist_bin_id] += 1;
    }

    for(size_t hist_bin_id=0; hist_bin_id < num_clusters; hist_bin_id++)
    {   //normalize such that sum(hist)=1
        hist_train [hist_bin_id] /= label_test.rows;
    }

    std::vector<std::vector<double> > data_test;
    data_test.push_back(hist_train);
    std::vector<double> target_pred;
    pSVM->SVMpredictTarget(data_test, target_pred);
    std::cout << "Predicted class " << target_pred[0] << std::endl;

    if(pInputCloud_->size() <= 0)
    {
        PCL_ERROR("No input cloud defined.");
    }
    else
    {
        pcl::PointCloud<ESFFeatureT>::CloudVectorType esf_signature;
        std::vector<Eigen::Vector3f> centroids;
        pcl::PointCloud<PointT>::Ptr pProcessedCloud;
        pProcessedCloud.reset(new pcl::PointCloud<PointT>());

        /*esf_estimator_->estimate(pInputCloud_, pProcessedCloud, esf_signature, centroids);
        int K = 10;

        std::vector<int> pointIdxNKNSearch(K);
        std::vector<float> pointNKNSquaredDistance(K);

        if ( kdtree_.nearestKSearch (esf_signature[0].points[0], K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
        {
            for (size_t i = 0; i < pointIdxNKNSearch.size (); ++i)
                std::cout << "  Class:  "  <<   models3D_->at(pointIdxNKNSearch[i])->class_
                          << "  Id:  "  <<   models3D_->at(pointIdxNKNSearch[i])->id_
                          << " (squared distance: " << pointNKNSquaredDistance[i] << ")" << std::endl;
        }*/
    }
}

void MyClassifier::testClassifier()
{
    size_t num_dimensions;

    boost::shared_ptr < faat_pcl::rec_3d_framework::Source2D> pImgSource;
    pImgSource.reset(new faat_pcl::rec_3d_framework::Source2D ());
    pImgSource->setPath(test_dir_);
    pImgSource->generate();
    boost::shared_ptr<std::vector<Model2DTPtr> > models2D = pImgSource->getModels();

    boost::shared_ptr < faat_pcl::rec_3d_framework::UnregisteredViewsSource < PointT >
            > pPCSource;
    pPCSource.reset( new faat_pcl::rec_3d_framework::UnregisteredViewsSource < PointT > ( ) );
    pPCSource->setPath( test_dir_ );
    std::string dummy = "";
    pPCSource->generate( dummy );
    boost::shared_ptr < faat_pcl::rec_3d_framework::Source<PointT> >cast_source =
            boost::static_pointer_cast<faat_pcl::rec_3d_framework::UnregisteredViewsSource<PointT> > (pPCSource);
    boost::shared_ptr<std::vector<ModelTPtr> > models3D = cast_source->getModels ();

    num_classes_ = class_map_.size();

    std::vector<std::vector<std::string> > view_name_per_class_v ( num_classes_ );
    std::vector<std::vector<cv::Mat> >  local_signatures_per_class_per_view_v ( num_classes_ );
    std::vector<std::vector<std::vector<double> > > global_signatures_per_class_per_view_v ( num_classes_ );

    for(size_t i=0; i < models3D->size(); i++)
    {
        cv::Mat FeatureMatrix;
        std::string class_name = models3D->at(i)->class_;
        std::string view_name = models3D->at(i)->id_;
        size_t class_id = class_map_[ class_name ];
        std::vector<double> glob_signature_tmp;//, merged_glob_signature;

        view_name_per_class_v[class_id].push_back(view_name);

        if((features3d_ & ESF_3D) == ESF_3D)
        {
            feature_extractor_->computeESFFeatures(models3D->at(i)->views_->at(0), models3D->at(i)->indices_->at(0), FeatureMatrix, num_dimensions);
            for(size_t feat_id = 0; feat_id < FeatureMatrix.cols; feat_id ++)
            {
                glob_signature_tmp.push_back(FeatureMatrix.at<float>(0,feat_id));
            }
        }

        if((features3d_ & CUSTOM_3D) == CUSTOM_3D)
        {
            feature_extractor_->computeCustomFeatures(models3D->at(i)->views_->at(0), models3D->at(i)->indices_->at(0), FeatureMatrix, num_dimensions);
            for(size_t feat_id = 0; feat_id < FeatureMatrix.cols; feat_id ++)
            {
                glob_signature_tmp.push_back(FeatureMatrix.at<float>(0,feat_id));
            }
        }

        if((features3d_ & SIFT_3D) == SIFT_3D)
        {
            feature_extractor_->computeSIFTFeatures(models3D->at(i)->views_->at(0), models3D->at(i)->indices_->at(0), FeatureMatrix, num_dimensions);
            local_signatures_per_class_per_view_v[class_id].push_back(FeatureMatrix);

            cv::Mat labels;
            std::vector<double> glob_signature_sift;//, merged_glob_signature;
            siftCodebook_->computeLabels(FeatureMatrix, labels);
            siftCodebook_->computeHist(labels, glob_signature_sift);
            for(size_t feat_id = 0; feat_id < glob_signature_sift.size(); feat_id ++)
            {
                glob_signature_tmp.push_back(glob_signature_sift[feat_id]);
            }
        }

        if((features3d_ & SHOT_3D) == SHOT_3D)
        {
            feature_extractor_->computeSHOTFeatures(models3D->at(i)->views_->at(0), models3D->at(i)->indices_->at(0), FeatureMatrix, num_dimensions);
            // :::::
            // :::::
            // :::::
        }

        //merged_glob_signature.reserve(glob_signature_tmp.size());
        //merged_glob_signature.insert (merged_glob_signature.end(), glob_signature_sift.begin() , glob_signature_sift.end() );
        //merged_glob_signature.insert (merged_glob_signature.end(), glob_signature_tmp.begin(), glob_signature_tmp.end() );

        global_signatures_per_class_per_view_v[class_id].push_back(glob_signature_tmp);
    }

    for(size_t i=0; i < models2D->size(); i++)
    {
        cv::Mat FeatureMatrix;
        std::string class_name = models2D->at(i)->class_;
        std::string view_name = models2D->at(i)->id_;
        size_t class_id = class_map_[class_name];
        std::vector<double> glob_signature_tmp;

        view_name_per_class_v[class_id].push_back(view_name);

        if((features2d_ & CUSTOM_2D) == CUSTOM_2D)
        {
            feature_extractor_->computeCustomFeatures(*(models2D->at(i)->view_), FeatureMatrix, num_dimensions);
            for(size_t feat_id = 0; feat_id < FeatureMatrix.cols; feat_id ++)
            {
                glob_signature_tmp.push_back(FeatureMatrix.at<float>(0,feat_id));
            }
        }

        if((features2d_ & SIFT_2D) == SIFT_2D)
        {
            std::cout << "computing SIFT for " << view_name << std::endl;
            feature_extractor_->computeSIFTFeatures(*(models2D->at(i)->view_), FeatureMatrix, num_dimensions);
            local_signatures_per_class_per_view_v[class_id].push_back(FeatureMatrix);

            cv::Mat labels;
            std::vector<double> glob_signature_sift;//, merged_glob_signature;
            siftCodebook_->computeLabels(FeatureMatrix, labels);
            siftCodebook_->computeHist(labels, glob_signature_sift);
            for(size_t feat_id = 0; feat_id < glob_signature_sift.size(); feat_id ++)
            {
                glob_signature_tmp.push_back(glob_signature_sift[feat_id]);
            }
        }

        global_signatures_per_class_per_view_v[class_id].push_back(glob_signature_tmp);
    }

    std::vector<double> predicted_class_label, actual_class_label;

    std::map<std::string, size_t>::const_iterator class_it;
    for (class_it = class_map_.begin(); class_it != class_map_.end(); class_it ++)
    {
        size_t class_id = class_it->second;

        for(size_t view_id=0; view_id < global_signatures_per_class_per_view_v[class_id].size(); view_id++)
        {
            svm::svm_node *svm_n_test = new svm::svm_node[global_signatures_per_class_per_view_v[class_id][view_id].size()+1];

            for(size_t feat_attr_id=0; feat_attr_id < global_signatures_per_class_per_view_v[class_id][view_id].size(); feat_attr_id++)
            {
                svm_n_test[feat_attr_id].value = global_signatures_per_class_per_view_v[class_id][view_id][feat_attr_id];
                svm_n_test[feat_attr_id].index = feat_attr_id+1;
            }
            svm_n_test[global_signatures_per_class_per_view_v[class_id][view_id].size()].index = -1;
            double prob[num_classes_];
            predicted_class_label.push_back(svm::svm_predict_probability(pSVM->svm_mod_, svm_n_test, prob));
            actual_class_label.push_back(class_id);
            std::cout << "Predicted class: " << predicted_class_label.back() << " with probability " <<
                      prob[static_cast<size_t>(predicted_class_label.back())]*100    << "% for class id: "
                      << class_id << "(" << class_it->first << ") and view: " << view_name_per_class_v[class_id][view_id] << std::endl;
            std::cout << "Probability estimates: " << std::endl;

            for(size_t i=0; i<num_classes_; i++)
            {
                std::cout << prob[i] << std::endl;
            }
        }
    }
    cv::Mat confusion_matrix_test;
    pSVM->computeConfusionMatrix(predicted_class_label, actual_class_label, confusion_matrix_test);

    size_t sum=0;
    size_t trace=0;
    for(size_t i=0; i<confusion_matrix_test.rows; i++)
    {
        for(size_t jj=0; jj<confusion_matrix_test.cols; jj++)
        {
            sum += confusion_matrix_test.at<unsigned short>(i,jj);
            if (i == jj)
                trace += confusion_matrix_test.at<unsigned short>(i,jj);
        }
    }
    double avg_performance = static_cast<double>(trace) / sum;
    std::cout << "Confusion Matrix: " << std::endl << confusion_matrix_test << std::endl
                 << "Average Performance: " << avg_performance << std::endl;
}



    /*pcl::PointCloud<ESFFeatureT>::CloudVectorType esf_signature;
    std::vector<Eigen::Vector3f> centroids;
    pcl::PointCloud<PointT>::Ptr pFilteredCloud, pProcessedCloud;
    pFilteredCloud.reset(new pcl::PointCloud<PointT>());
    pProcessedCloud.reset(new pcl::PointCloud<PointT>());
    pcl::copyPointCloud(*(models3D_->at(i)->views_->at(0)), indices, *pFilteredCloud);
    //std::vector<PointT> test = pFilteredCloud->points();
    esf_estimator_->estimate(pFilteredCloud, pProcessedCloud, esf_signature, centroids);
    for(int kk=0; kk<640; kk++)
    {
        esf_signatures_->points[i].histogram[kk] = esf_signature[0].points[0].histogram[kk];
    }*/

    /*Eigen::Vector4f centroid;
    Eigen::Matrix3f covariance_matrix;
    pcl::PointCloud<PointT>::Ptr pClusterPCl_transformed (new pcl::PointCloud<PointT>());
    pcl::computeMeanAndCovarianceMatrix(*(models3D->at(i)->views_->at(0)), covariance_matrix, centroid);
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

    pcl::transformPointCloud(*(models3D->at(i)->views_->at(0)), *pClusterPCl_transformed, transformation_matrix);

    //pcl::transformPointCloud(*frame_, cluster_indices_int, *frame_eigencoordinates_, eigvects);
    PointT min_pt, max_pt;
    pcl::getMinMax3D(*pClusterPCl_transformed, min_pt, max_pt);
    std::cout << "Elongations along eigenvectors: " << max_pt.x - min_pt.x << ", " << max_pt.y - min_pt.y
              << ", " << max_pt.z - min_pt.z << std::endl;*/
