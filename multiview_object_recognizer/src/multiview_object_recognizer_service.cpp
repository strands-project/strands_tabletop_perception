#define EIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET

#include "multiview_object_recognizer_service.h"
#include "boost_graph_extension.h"
#include <pcl/common/transforms.h>
#include <faat_pcl/utils/pcl_visualization_utils.h>
#include <faat_pcl/utils/segmentation_utils.h>
#include <faat_pcl/utils/filesystem_utils.h>
#include "geometry_msgs/Point32.h"
#include "geometry_msgs/Transform.h"
#include "std_msgs/Time.h"

//std::string multiviewGraph::models_dir() const
//{
//    return models_dir_;
//}

//void multiviewGraph::setModels_dir(const std::string &models_dir)
//{
//    models_dir_ = models_dir;
//}

bool multiviewGraph::visualize_output() const
{
    return visualize_output_;
}

void multiviewGraph::setVisualize_output(bool visualize_output)
{
    visualize_output_ = visualize_output;
}

//bool multiviewGraph::go_3d() const
//{
//    return go_3d_;
//}

//void multiviewGraph::setGo_3d(bool go_3d)
//{
//    go_3d_ = go_3d;
//}

int multiviewGraph::icp_iter() const
{
    return icp_iter_;
}

void multiviewGraph::setIcp_iter(int icp_iter)
{
    icp_iter_ = icp_iter;
}

//int multiviewGraph::opt_type() const
//{
//    return opt_type_;
//}

std::string multiviewGraph::gt_or_ouput_dir() const
{
    return gt_or_ouput_dir_;
}

void multiviewGraph::setGt_or_ouput_dir(const std::string &gt_or_ouput_dir)
{
    gt_or_ouput_dir_ = gt_or_ouput_dir;
}

double multiviewGraph::chop_at_z() const
{
    return chop_at_z_;
}

void multiviewGraph::setChop_at_z(double chop_at_z)
{
    chop_at_z_ = chop_at_z;
}

//int multiviewGraph::mv_keypoints() const
//{
//    return mv_keypoints_;
//}

//void multiviewGraph::setMv_keypoints(int mv_keypoints)
//{
//    mv_keypoints_ = mv_keypoints;
//}

//void multiviewGraph::loadModels()
//{
//load models for visualization
//    models_source_.reset ( new faat_pcl::rec_3d_framework::ModelOnlySource<pcl::PointXYZRGBNormal, PointT> );
//    models_source_->setPath ( models_dir_ );
//    models_source_->setLoadViews ( false );
//    models_source_->setLoadIntoMemory ( false );
//    std::string training_dir = "not_needed";
//    models_source_->generate ( training_dir );
//    models_source_->createVoxelGridAndDistanceTransform ( icp_resolution_ );
//    ROS_INFO ( "Models loaded from %s", models_dir_.c_str() );
//}



void multiviewGraph::setPSingleview_recognizer(const boost::shared_ptr<Recognizer> &value)
{
    pSingleview_recognizer_ = value;
}


cv::Ptr<SiftGPU> multiviewGraph::sift() const
{
    return sift_;
}

void multiviewGraph::setSift(const cv::Ptr<SiftGPU> &sift)
{
    sift_ = sift;
}


bool multiviewGraph::
calcSiftFeatures (Vertex &src, Graph &grph)
{
    boost::shared_ptr < faat_pcl::rec_3d_framework::SIFTLocalEstimation<PointT, FeatureT> > estimator;
    estimator.reset (new faat_pcl::rec_3d_framework::SIFTLocalEstimation<PointT, FeatureT>(sift_));

    //    if(use_table_plane)
    //        estimator->setIndices (*(grph[src].pIndices_above_plane));

    boost::shared_ptr< pcl::PointCloud<PointT> > pSiftKeypoints;
    bool ret = estimator->estimate (grph[src].pScenePCl_f, pSiftKeypoints, grph[src].pSiftSignatures_, grph[src].sift_keypoints_scales);
    estimator->getKeypointIndices(grph[src].siftKeypointIndices_);

    return ret;

    //----display-keypoints--------------------
    /*pcl::visualization::PCLVisualizer::Ptr vis_temp (new pcl::visualization::PCLVisualizer);
     pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> handler_rgb_verified (grph[*it_vrtx].pScenePCl);
     vis_temp->addPointCloud<pcl::PointXYZRGB> (grph[*it_vrtx].pScenePCl, handler_rgb_verified, "Hypothesis_1");
     pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> handler_rgb_verified2 (grph[*it_vrtx].pSiftKeypoints);

     for (size_t keyId = 0; keyId < grph[*it_vrtx].pSiftKeypoints->size (); keyId++)
     {
     std::stringstream sphere_name;
     sphere_name << "sphere_" << keyId;
     vis_temp->addSphere<pcl::PointXYZRGB> (grph[*it_vrtx].pSiftKeypoints->at (keyId), 0.01, sphere_name.str ());
     }
     vis_temp->spin ();*/
}

void
multiview::nearestKSearch ( flann::Index<flann::L1<float> > * index, float * descr, int descr_size, int k, flann::Matrix<int> &indices,
                            flann::Matrix<float> &distances )
{
    flann::Matrix<float> p = flann::Matrix<float> ( new float[descr_size], 1, descr_size );
    memcpy ( &p.ptr () [0], &descr[0], p.cols * p.rows * sizeof ( float ) );

    index->knnSearch ( p, indices, distances, k, flann::SearchParams ( 128 ) );
    delete[] p.ptr ();
}

template<typename Type>
void
multiview::convertToFLANN ( typename pcl::PointCloud<Type>::Ptr & cloud, flann::Matrix<float> &data )
{
    data.rows = cloud->points.size ();
    data.cols = sizeof ( cloud->points[0].histogram ) / sizeof ( float ); // number of histogram bins

    std::cout << data.rows << " " << data.cols << std::endl;

    flann::Matrix<float> flann_data ( new float[data.rows * data.cols], data.rows, data.cols );

    for ( size_t i = 0; i < data.rows; ++i )
        for ( size_t j = 0; j < data.cols; ++j )
        {
            flann_data.ptr () [i * data.cols + j] = cloud->points[i].histogram[j];
        }

    data = flann_data;
}

template void
multiview::convertToFLANN<pcl::Histogram<128> > ( pcl::PointCloud<pcl::Histogram<128> >::Ptr & cloud, flann::Matrix<float> &data ); // explicit instantiation.


void multiviewGraph::
estimateViewTransformationBySIFT ( const Vertex &src, const Vertex &trgt, Graph &grph, flann::Index<DistT> *flann_index, Eigen::Matrix4f &transformation,
                                   std::vector<Edge> & edges, bool use_gc )
{
    const int K = 1;
    flann::Matrix<int> indices = flann::Matrix<int> ( new int[K], 1, K );
    flann::Matrix<float> distances = flann::Matrix<float> ( new float[K], 1, K );

    boost::shared_ptr< pcl::PointCloud<PointT> > pSiftKeypointsSrc (new pcl::PointCloud<PointT>);
    boost::shared_ptr< pcl::PointCloud<PointT> > pSiftKeypointsTrgt (new pcl::PointCloud<PointT>);
    pcl::copyPointCloud(*(grph[ src].pScenePCl_f), grph[ src].siftKeypointIndices_, *(pSiftKeypointsSrc ));
    pcl::copyPointCloud(*(grph[trgt].pScenePCl_f), grph[trgt].siftKeypointIndices_, *(pSiftKeypointsTrgt));
    PCL_INFO ( "Calculate transform via SIFT between view %s and %s for a keypoint size of %ld (src) and %ld (target).",
               grph[src].pScenePCl->header.frame_id.c_str(), grph[trgt].pScenePCl->header.frame_id.c_str(), pSiftKeypointsSrc->points.size(), pSiftKeypointsTrgt->points.size() );

    pcl::CorrespondencesPtr temp_correspondences ( new pcl::Correspondences );
    for ( size_t keypointId = 0; keypointId < pSiftKeypointsSrc->size (); keypointId++ )
    {
        FeatureT searchFeature = grph[src].pSiftSignatures_->at ( keypointId );
        int size_feat = sizeof ( searchFeature.histogram ) / sizeof ( float );
        multiview::nearestKSearch ( flann_index, searchFeature.histogram, size_feat, K, indices, distances );

        pcl::Correspondence corr;
        corr.distance = distances[0][0];
        corr.index_query = keypointId;
        corr.index_match = indices[0][0];
        temp_correspondences->push_back ( corr );
    }

    if(!use_gc)
    {
        pcl::registration::CorrespondenceRejectorSampleConsensus<PointT>::Ptr rej;
        rej.reset (new pcl::registration::CorrespondenceRejectorSampleConsensus<PointT> ());
        pcl::CorrespondencesPtr after_rej_correspondences (new pcl::Correspondences ());

        rej->setMaximumIterations (50000);
        rej->setInlierThreshold (0.02);
        rej->setInputTarget (pSiftKeypointsTrgt);
        rej->setInputSource (pSiftKeypointsSrc);
        rej->setInputCorrespondences (temp_correspondences);
        rej->getCorrespondences (*after_rej_correspondences);

        transformation = rej->getBestTransformation ();
        pcl::registration::TransformationEstimationSVD<PointT, PointT> t_est;
        t_est.estimateRigidTransformation (*pSiftKeypointsSrc, *pSiftKeypointsTrgt, *after_rej_correspondences, transformation);

        std::cout << "size of corr before " << temp_correspondences->size () << "; after: " << after_rej_correspondences->size () << std::endl;

        bool b;
        Edge edge;
        tie (edge, b) = add_edge (trgt, src, grph);
        grph[edge].transformation = transformation;
        grph[edge].model_name = std::string ("scene_to_scene");
        grph[edge].source_id = grph[src].pScenePCl->header.frame_id;
        grph[edge].target_id = grph[trgt].pScenePCl->header.frame_id;
        edges.push_back(edge);
    }
    else
    {
        pcl::GeometricConsistencyGrouping<pcl::PointXYZRGB, pcl::PointXYZRGB> gcg_alg;

        gcg_alg.setGCThreshold (15);
        gcg_alg.setGCSize (0.01);
        //        if( (pSiftKeypointsSrc->points.size() == grph[src].keypointIndices_.indices.size())
        //                && (pSiftKeypointsTrgt->points.size() == grph[trgt].keypointIndices_.indices.size()) )
        //        {
        //            std::cout << "we could use normals " << std::endl;
        //            pcl::PointCloud<pcl::Normal>::Ptr pInputNormals (new pcl::PointCloud<pcl::Normal>);
        //            pcl::PointCloud<pcl::Normal>::Ptr pSceneNormals (new pcl::PointCloud<pcl::Normal>);
        //            pcl::copyPointCloud(*(grph[ src].pSceneNormals_f_), grph[ src].keypointIndices_, *pInputNormals);
        //            pcl::copyPointCloud(*(grph[trgt].pSceneNormals_f_), grph[trgt].keypointIndices_, *pSceneNormals);
        //            gcg_alg.setInputAndSceneNormals(pInputNormals, pSceneNormals);
        //        }
        gcg_alg.setInputCloud(pSiftKeypointsSrc);
        gcg_alg.setSceneCloud(pSiftKeypointsTrgt);
        gcg_alg.setModelSceneCorrespondences(temp_correspondences);

        std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > transformations;
        std::vector<pcl::Correspondences> clustered_corrs;
        gcg_alg.recognize(transformations, clustered_corrs);

        for(size_t i=0; i < transformations.size(); i++)
        {
            //std::cout << clustered_corrs[i].size() << std::endl;
            PointInTPtr transformed_PCl (new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::transformPointCloud (*grph[src].pScenePCl, *transformed_PCl, transformations[i]);

            std::stringstream scene_stream;
            scene_stream << "scene_to_scene_cg_" << i;
            bool b;
            Edge edge;
            tie (edge, b) = add_edge (trgt, src, grph);
            grph[edge].transformation = transformations[i];
            grph[edge].model_name = scene_stream.str();
            grph[edge].source_id = grph[src].pScenePCl->header.frame_id;
            grph[edge].target_id = grph[trgt].pScenePCl->header.frame_id;
            edges.push_back(edge);
        }
    }
}


void multiviewGraph::visualizeEdge (const Edge &edge, const Graph &grph)
{
    Vertex src = source ( edge, grph );
    Vertex trgt = target ( edge, grph );

    Eigen::Matrix4f transform;

    if ( grph[edge].source_id.compare( grph[src].pScenePCl->header.frame_id ) == 0)
    {
        transform = grph[edge].transformation;
    }
    else if (grph[edge].target_id.compare( grph[src].pScenePCl->header.frame_id ) == 0)
    {
        transform = grph[edge].transformation.inverse();
    }
    else
    {
        std::cout << "Something is messed up with the transformation! " << std::endl;
    }

    if(!edge_vis_)
        edge_vis_.reset (new pcl::visualization::PCLVisualizer());

    edge_vis_->removeAllPointClouds();
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> handler_rgb_verified (grph[trgt].pScenePCl_f);
    edge_vis_->addPointCloud<pcl::PointXYZRGB> (grph[trgt].pScenePCl_f, handler_rgb_verified, "Hypothesis_1");
    PointInTPtr transformed_PCl (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::transformPointCloud (*grph[src].pScenePCl_f, *transformed_PCl, transform);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> handler_rgb_verified2 (transformed_PCl);
    edge_vis_->addPointCloud<pcl::PointXYZRGB> (transformed_PCl, handler_rgb_verified2, "Hypothesis_2");
    std::stringstream window_title;
    window_title << "transform of source view_id " << grph[src].pScenePCl->header.frame_id << " to target view_id " << grph[trgt].pScenePCl->header.frame_id << " with edge " << grph[edge].model_name;
    edge_vis_->setWindowName(window_title.str());
    edge_vis_->spin ();
}

/*
 * If a global_transform for the point cloud is provided as an argument for the recognition
 * service and use_robot_pose is set to true, additional edges between each views (vertices) are created.
 */
void multiviewGraph::
estimateViewTransformationByRobotPose ( const Vertex &src, const Vertex &trgt, Graph &grph, Edge &edge )
{
    bool b;
    tie ( edge, b ) = add_edge ( trgt, src, grph );
    Eigen::Matrix4f tf2wco_src = grph[src].transform_to_world_co_system_;
    Eigen::Matrix4f tf2wco_trgt = grph[trgt].transform_to_world_co_system_;
    grph[edge].transformation = tf2wco_trgt.inverse() * tf2wco_src;
    grph[edge].model_name = std::string ( "robot_pose" );
    grph[edge].source_id = grph[src].pScenePCl->header.frame_id;
    grph[edge].target_id = grph[trgt].pScenePCl->header.frame_id;
}



/*
 * Transfers keypoints and hypotheses from other views (vertices) in the graph and
 * adds/merges them to the current keyppoints (existing) ones.
 * Correspondences with keypoints close to existing ones (distance, normal and model id) are
 * not transferred (to avoid redundant correspondences)
 */
void multiviewGraph::
extendFeatureMatchesRecursive ( Graph &grph,
                                Vertex &vrtx_start,
                                std::map < std::string,faat_pcl::rec_3d_framework::ObjectHypothesis<PointT> > &hypotheses,
                                pcl::PointCloud<PointT>::Ptr pKeypoints,
                                pcl::PointCloud<pcl::Normal>::Ptr pKeypointNormals)
{
    //    hypotheses = grph[vrtx_start].hypotheses_;
    pcl::copyPointCloud(*(grph[vrtx_start].pKeypointsMultipipe_), *pKeypoints);
    //    pcl::PointCloud<pcl::Normal> normals_f;
    //    pcl::copyPointCloud(*(grph[vrtx_start].pSceneNormals), grph[vrtx_start].filteredSceneIndices_, normals_f);
    pcl::copyPointCloud(*(grph[vrtx_start].pKeypointNormalsMultipipe_), *pKeypointNormals);

    std::map<std::string, faat_pcl::rec_3d_framework::ObjectHypothesis<PointT> >::const_iterator it_copy_hyp;
    for(it_copy_hyp = grph[vrtx_start].hypotheses_.begin(); it_copy_hyp !=grph[vrtx_start].hypotheses_.end(); ++it_copy_hyp)
    {
        faat_pcl::rec_3d_framework::ObjectHypothesis<PointT> oh;

        oh.model_ = it_copy_hyp->second.model_;
        oh.correspondences_pointcloud.reset(new pcl::PointCloud<PointT>);
        pcl::copyPointCloud(*(it_copy_hyp->second.correspondences_pointcloud), *(oh.correspondences_pointcloud));
        oh.normals_pointcloud.reset(new pcl::PointCloud<pcl::Normal>);
        pcl::copyPointCloud(*(it_copy_hyp->second.normals_pointcloud), *(oh.normals_pointcloud));
        oh.feature_distances_.reset(new std::vector<float>);
        *(oh.feature_distances_) = *(it_copy_hyp->second.feature_distances_);
        oh.correspondences_to_inputcloud.reset(new pcl::Correspondences);
        *(oh.correspondences_to_inputcloud) = *(it_copy_hyp->second.correspondences_to_inputcloud);
        oh.num_corr_ = it_copy_hyp->second.num_corr_;
        oh.indices_to_flann_models_ = it_copy_hyp->second.indices_to_flann_models_;

        hypotheses.insert(std::pair<std::string, faat_pcl::rec_3d_framework::ObjectHypothesis<PointT> >(it_copy_hyp->first, oh));
    }

    assert(pKeypoints->points.size() == pKeypointNormals->points.size());

    //    std::stringstream viewer_original_name;
    //    viewer_original_name << "original keypoints for view id " << grph[vrtx_start].pScenePCl->header.frame_id.c_str();
    //    pcl::visualization::PCLVisualizer viewer(viewer_original_name.str());
    //    viewer.removeAllPointClouds();
    //    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> handler_rgb (grph[vrtx_start].pScenePCl_f);
    //    viewer.addPointCloud<pcl::PointXYZRGB>(grph[vrtx_start].pScenePCl_f, handler_rgb, "scene");
    //    viewer.addPointCloudNormals<pcl::PointXYZRGB,pcl::Normal>(pKeypoints, pKeypointNormals, 3, 0.03, "original");


    grph[vrtx_start].has_been_hopped_ = true;

    typename graph_traits<Graph>::out_edge_iterator out_i, out_end;
    tie ( out_i, out_end ) = out_edges ( vrtx_start, grph);
//    size_t num_edges = std::distance(out_i, out_end);
    if (out_i == out_end) // This vertex does not have any more edge to hop
    {
        return;
    }
    else    //get hypotheses from next vertex. Just taking the first one not being hopped.
    {
        std::string edge_src, edge_trgt;
        Vertex remote_vertex;
        bool have_found_a_valid_edge = false;
        for (; out_i != out_end; ++out_i )
        {
            remote_vertex = target ( *out_i, grph );
            edge_src = grph[*out_i].source_id;
            edge_trgt = grph[*out_i].target_id;

            if ( grph[remote_vertex].has_been_hopped_ )
            {
                ROS_INFO("Vertex %s has already been hopped.", grph[remote_vertex].pScenePCl->header.frame_id.c_str());
            }
            else
            {
                have_found_a_valid_edge = true;
                std::cout << "Edge src: " << edge_src << "; target: " << edge_trgt << "; model_name: " << grph[*out_i].model_name
                          << "; edge_weight: " << grph[*out_i].edge_weight << std::endl;
                break;
            }
        }
        if(!have_found_a_valid_edge)
        {
            std::cout << "This vertex does not have any more edge to hop. Just added my hypotheses." << std::endl;
            return;
        }
        ROS_INFO("Hopping to vertex %s...", grph[remote_vertex].pScenePCl->header.frame_id.c_str());


        std::map<std::string, faat_pcl::rec_3d_framework::ObjectHypothesis<PointT> > new_hypotheses;
        pcl::PointCloud<PointT>::Ptr pNewKeypoints (new pcl::PointCloud<PointT>);
        pcl::PointCloud<pcl::Normal>::Ptr pNewKeypointNormals (new pcl::PointCloud<pcl::Normal>);
        extendFeatureMatchesRecursive ( grph, remote_vertex, new_hypotheses, pNewKeypoints, pNewKeypointNormals);
        assert( pNewKeypoints->size() == pNewKeypointNormals->size() );

        //------check for right transformation between the two vertices (edges are undirected, so we have to check the source/target attribute of the edge)------
        Eigen::Matrix4f transform;
        if ( edge_src.compare( grph[vrtx_start].pScenePCl->header.frame_id ) == 0)
        {
            transform = grph[*out_i].transformation.inverse ();

        }
        else if (edge_trgt.compare( grph[vrtx_start].pScenePCl->header.frame_id ) == 0)
        {
            transform = grph[*out_i].transformation;
        }
        else
        {
            ROS_WARN("Something is messed up with the transformation.");
        }



        //------ Transform keypoints and rotate normals----------
        Eigen::Matrix3f rot   = transform.block<3, 3> (0, 0);
        Eigen::Vector3f trans = transform.block<3, 1> (0, 3);
        for (size_t i=0; i<pNewKeypoints->size(); i++)
        {
            pNewKeypoints->points[i].getVector3fMap () = rot * pNewKeypoints->points[i].getVector3fMap () + trans;
            pNewKeypointNormals->points[i].getNormalVector3fMap() = rot * pNewKeypointNormals->points[i].getNormalVector3fMap ();
        }


        // add/merge hypotheses
        // nearest neighbor search to avoid "duplicate" keypoints
        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
        pcl::PointCloud<pcl::PointXYZ>::Ptr pKeypointsXYZ (new pcl::PointCloud<pcl::PointXYZ>);
        for(size_t i=0; i < pKeypoints->size(); i++)
        {
            pcl::PointXYZ tmp_pt;
            tmp_pt.x = pKeypoints->points[i].x;
            tmp_pt.y = pKeypoints->points[i].y;
            tmp_pt.z = pKeypoints->points[i].z;
            pKeypointsXYZ->points.push_back(tmp_pt);
        }
        kdtree.setInputCloud(pKeypointsXYZ);
        int K =1;
        std::vector<int> pointIdxNKNSearch(K);
        std::vector<float> pointNKNSquaredDistance(K);

        std::map<std::string, faat_pcl::rec_3d_framework::ObjectHypothesis<PointT> >::iterator it_new_hyp;

        //        pcl::visualization::PCLVisualizer viewer("Keypoint Viewer");
        for(it_new_hyp = new_hypotheses.begin(); it_new_hyp !=new_hypotheses.end(); ++it_new_hyp)
        {
            //            viewer.removeAllPointClouds();
            //            ConstPointInTPtr model_cloud = it_new_hyp->second.model_->getAssembled (0.003f);
            //            pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> handler_rgb (model_cloud);
            //            viewer.addPointCloud<pcl::PointXYZRGB>(model_cloud, handler_rgb, "scene");
            //            viewer.addPointCloudNormals<pcl::PointXYZRGB,pcl::Normal>(it_new_hyp->second.correspondences_pointcloud, it_new_hyp->second.normals_pointcloud, 5, 0.04);
            //            viewer.spin();


            std::string id = it_new_hyp->second.model_->id_;
            std::cout << "Adding/Merging hypotheses for " << id << "." << std::endl;

            std::map<std::string, faat_pcl::rec_3d_framework::ObjectHypothesis<PointT> >::iterator it_existing_hyp;
            it_existing_hyp = hypotheses.find(id);
            if (it_existing_hyp == hypotheses.end())
            {
                std::cout << "There are no hypotheses (feature matches) for this model yet." << std::endl;
                for(size_t kk=0; kk < it_new_hyp->second.correspondences_to_inputcloud->size(); kk++)
                {
                    it_new_hyp->second.correspondences_to_inputcloud->at(kk).index_match += pKeypoints->points.size();
                }
                hypotheses.insert(std::pair<std::string, faat_pcl::rec_3d_framework::ObjectHypothesis<PointT> >(id, it_new_hyp->second));
            }
            else
            { //merge hypotheses
                std::cout << "INFO: Size for " << id << " of correspondes_pointcloud old: " << it_existing_hyp->second.correspondences_pointcloud->points.size()
                          << "  and new: " << it_new_hyp->second.correspondences_pointcloud->points.size() << std::endl;

                for(size_t kk=0; kk < it_new_hyp->second.correspondences_to_inputcloud->size(); kk++)
                {
                    bool drop_new_correspondence = false;

                    pcl::Correspondence c_new = it_new_hyp->second.correspondences_to_inputcloud->at(kk);
                    pcl::PointXYZ searchPoint;
                    searchPoint.x = pNewKeypoints->points[c_new.index_match].x;
                    searchPoint.y = pNewKeypoints->points[c_new.index_match].y;
                    searchPoint.z = pNewKeypoints->points[c_new.index_match].z;

                    PointT model_point_new = it_new_hyp->second.correspondences_pointcloud->points[ c_new.index_query ];
                    pcl::PointXYZ model_point_newXYZ;
                    model_point_newXYZ.x = model_point_new.x;
                    model_point_newXYZ.y = model_point_new.y;
                    model_point_newXYZ.z = model_point_new.z;

                    if (!pcl_isfinite (searchPoint.x) || !pcl_isfinite (searchPoint.y) || !pcl_isfinite (searchPoint.z))
                    {
                        std::cerr << "Search point is infinity!!" << std::endl;
                        drop_new_correspondence = true;
                        continue;
                    }

                    if ( kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
                    {
                        if(pointNKNSquaredDistance[0] < distance_keypoints_get_discarded_);
                        {
                            Eigen::Vector3f searchNormal = pNewKeypointNormals->points[c_new.index_match].getNormalVector3fMap();
                            Eigen::Vector3f nnNormal = pKeypointNormals->points[pointIdxNKNSearch[0]].getNormalVector3fMap();

                            //check if the keypoint is linked with the same model
                            for(size_t j=0; j < it_existing_hyp->second.correspondences_to_inputcloud->size(); j++)
                            {
                                pcl::Correspondence c_existing = it_existing_hyp->second.correspondences_to_inputcloud->at(j);
                                PointT model_point_exist = it_existing_hyp->second.correspondences_pointcloud->points[ c_new.index_query ];
                                pcl::PointXYZ model_point_existXYZ;
                                model_point_existXYZ.x = model_point_exist.x;
                                model_point_existXYZ.y = model_point_exist.y;
                                model_point_existXYZ.z = model_point_exist.z;

                                float squaredDistModelKeypoints = pcl::squaredEuclideanDistance(model_point_newXYZ, model_point_existXYZ);
                                //                            std::cout << "Squared Dist Model (id: " << it_existing_hyp->first << ", " << it_new_hyp->first << " ) Keypoints: " << squaredDistModelKeypoints << std::endl;

                                if((c_existing.index_match == pointIdxNKNSearch[0]) && (searchNormal.dot(nnNormal)>0.8) && squaredDistModelKeypoints < distance_keypoints_get_discarded_)
                                {
                                    std::cout << "Found a very close point (keypoint distance: " << pointNKNSquaredDistance[0]
                                              << "; model distance: " << squaredDistModelKeypoints
                                              << ") with the same model id and similar normal (Normal dot product: "
                                              << searchNormal.dot(nnNormal) << "> 0.8). Ignoring it."
                                              << std::endl;
                                    drop_new_correspondence = true;
                                    break;
                                }
                            }
                        }
                    }
                    if (!drop_new_correspondence)
                    {
                        c_new.index_match += pKeypoints->points.size();
                        c_new.index_query += it_existing_hyp->second.correspondences_pointcloud->points.size();
                        it_existing_hyp->second.correspondences_to_inputcloud->push_back(c_new);
                    }
                }

                *it_existing_hyp->second.correspondences_pointcloud += * it_new_hyp->second.correspondences_pointcloud;
                *it_existing_hyp->second.normals_pointcloud += * it_new_hyp->second.normals_pointcloud;
                it_existing_hyp->second.feature_distances_->insert(it_existing_hyp->second.feature_distances_->end(),
                                                                   it_existing_hyp->second.feature_distances_->begin(),
                                                                   it_existing_hyp->second.feature_distances_->end());
                it_existing_hyp->second.num_corr_ += it_new_hyp->second.num_corr_;

                std::cout << "INFO: Size for " << id << " of correspondes_pointcloud after merge: " << it_existing_hyp->second.correspondences_pointcloud->points.size() << std::endl;
            }
        }
        *pKeypoints += *pNewKeypoints;
        *pKeypointNormals += *pNewKeypointNormals;

        //        viewer.addPointCloudNormals<pcl::PointXYZRGB,pcl::Normal>(pNewKeypoints, pNewKeypointNormals, 3, 0.08, "new_keypoint_cloud");
    }
    //    viewer.spin();

    //    std::stringstream viewer2_name;
    //    viewer2_name << "extended keypoints for view id " << grph[vrtx_start].pScenePCl->header.frame_id.c_str();
    //    pcl::visualization::PCLVisualizer viewer2(viewer2_name.str());
    //    viewer2.removeAllPointClouds();
    //    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> handler_rgb2 (grph[vrtx_start].pScenePCl_f);
    //    viewer2.addPointCloud<pcl::PointXYZRGB>(grph[vrtx_start].pScenePCl_f, handler_rgb2, "scene");
    //    viewer2.addPointCloudNormals<pcl::PointXYZRGB,pcl::Normal>(pKeypoints, pKeypointNormals, 3, 0.04);
    //    viewer2.spin();
}


void multiviewGraph::
extendHypothesisRecursive ( Graph &grph, Edge calling_out_edge, std::vector<Hypothesis<PointT> > &hyp_vec) //is directed edge (so the source of calling_edge is calling vertex)
{
    typename graph_traits<Graph>::out_edge_iterator out_i, out_end;
    Vertex current_vertex = target ( calling_out_edge, grph );
    Vertex src = source ( calling_out_edge, grph );

    grph[current_vertex].has_been_hopped_ = true;
    grph[current_vertex].cumulative_weight_to_new_vrtx_ = grph[src].cumulative_weight_to_new_vrtx_ + grph[calling_out_edge].edge_weight;

    ROS_INFO("Current Vertex %s has a cumulative weight of %lf.", grph[current_vertex].pScenePCl->header.frame_id.c_str(), grph[current_vertex].cumulative_weight_to_new_vrtx_);
    for ( tie ( out_i, out_end ) = out_edges ( current_vertex, grph ); out_i != out_end; ++out_i )
    {
        Vertex new_trgt = target ( *out_i, grph );

        if ( grph[new_trgt].has_been_hopped_ )
        {
            ROS_INFO("Vertex %s has already been hopped.", grph[new_trgt].pScenePCl->header.frame_id.c_str());
            continue;
        }
        ROS_INFO("Hopping to vertex %s...", grph[new_trgt].pScenePCl->header.frame_id.c_str());
        std::vector<Hypothesis<PointT> > new_hypotheses;
        extendHypothesisRecursive ( grph, *out_i, new_hypotheses);
        for(std::vector<Hypothesis<PointT> >::iterator it_new_hyp = new_hypotheses.begin(); it_new_hyp !=new_hypotheses.end(); ++it_new_hyp)
        {
            if ( grph[calling_out_edge].source_id.compare( grph[src].pScenePCl->header.frame_id ) == 0)
            {
                it_new_hyp->transform_ = grph[calling_out_edge].transformation.inverse () * it_new_hyp->transform_ ;
            }
            else if (grph[calling_out_edge].target_id.compare( grph[src].pScenePCl->header.frame_id ) == 0)
            {
                it_new_hyp->transform_ = grph[calling_out_edge].transformation * it_new_hyp->transform_;
            }
            else
            {
                ROS_WARN("Something is messed up with the transformation.");
            }
            hyp_vec.push_back(*it_new_hyp);
        }
    }

    for ( std::vector<Hypothesis<PointT>>::const_iterator it_hyp = grph[current_vertex].hypothesis.begin (); it_hyp != grph[current_vertex].hypothesis.end (); ++it_hyp )
    {
        if(!it_hyp->verified_)
            continue;

        Eigen::Matrix4f tf;
        if ( grph[calling_out_edge].source_id.compare( grph[src].pScenePCl->header.frame_id ) == 0)
        {
            tf = grph[calling_out_edge].transformation.inverse () * it_hyp->transform_;
        }
        else if (grph[calling_out_edge].target_id.compare( grph[src].pScenePCl->header.frame_id ) == 0)
        {
            tf = grph[calling_out_edge].transformation * it_hyp->transform_;
        }
        else
        {
            ROS_WARN("Something is messed up with the transformation.");
        }
        Hypothesis<PointT> ht_temp ( it_hyp->model_id_, tf, it_hyp->origin_, true );
        hyp_vec.push_back(ht_temp);
    }
}

void multiviewGraph::visualizeGraph(const Graph &grph, pcl::visualization::PCLVisualizer::Ptr &vis)
{
    //--(bottom: Scene; 2nd from bottom: Single-view-results; 2nd from top: transformed hypotheses; top: verified hypotheses coming from all views)--
    //...

    std::vector<int> viewportNr;
    size_t vis_rows = 5;

    if ( !vis ) //-------Visualize Scene Cloud--------------------
    {
        vis.reset ( new pcl::visualization::PCLVisualizer ( "vis1" ) );
        vis->setWindowName ( "Recognition from Multiple Views" );
    }
    vis->removeAllPointClouds();
    viewportNr = faat_pcl::utils::visualization_framework ( vis, num_vertices(grph), vis_rows);

    std::pair<vertex_iter, vertex_iter> vp;
    int view_id = -1;
    for ( vp = vertices ( grph ); vp.first != vp.second; ++vp.first )
    {
        view_id++;
        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> handler_rgb ( grph[*vp.first].pScenePCl_f );
        std::stringstream cloud_name;
        cloud_name << "view_cloud_" << grph[*vp.first].pScenePCl->header.frame_id;
        vis->addPointCloud<pcl::PointXYZRGB> ( grph[*vp.first].pScenePCl_f, handler_rgb, cloud_name.str (), viewportNr[view_id * vis_rows + 0] );

        for ( size_t hyp_id = 0; hyp_id < grph[*vp.first].hypothesis.size(); hyp_id++ )
        {
            //visualize models
            //            std::string model_id = grph[*vp.first].hypothesis[hyp_id].model_id_;
            Eigen::Matrix4f trans = grph[*vp.first].hypothesis[hyp_id].transform_;
            ModelTPtr model = grph[*vp.first].hypothesis[hyp_id].model_;

            std::stringstream name;
            name << cloud_name.str() << "_sv__hypothesis_" << hyp_id;

            typename pcl::PointCloud<PointT>::Ptr model_aligned ( new pcl::PointCloud<PointT> );
            //            if(model)
            //            {
            ConstPointInTPtr model_cloud = model->getAssembled (0.005f);
            pcl::transformPointCloud (*model_cloud, *model_aligned, trans);
            //            }
            //            else
            //            {
            //                typename pcl::PointCloud<PointT>::Ptr pModelPCl ( new pcl::PointCloud<PointT> );
            //                std::stringstream model_name;
            //                model_name << models_dir_ << model_id;
            //                pcl::io::loadPCDFile ( model_name.str(), *pModelPCl );
            //                pcl::transformPointCloud ( *pModelPCl, *model_aligned, trans );
            //            }

            pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb_handler1 ( model_aligned );
            vis->addPointCloud<PointT> ( model_aligned, rgb_handler1, name.str (), viewportNr[view_id * vis_rows + 1] );

            if ( grph[*vp.first].hypothesis[hyp_id].verified_ )	//--show-verified-extended-hypotheses
            {
                name << "__verified";
                pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb_handler2 ( model_aligned );
                vis->addPointCloud<PointT> ( model_aligned, rgb_handler2, name.str (), viewportNr[view_id * vis_rows + 2] );
            }
        }

        for ( size_t hyp_id = 0; hyp_id < grph[*vp.first].hypothesis_mv_.size(); hyp_id++ )
        {
            //visualize models
            //            std::string model_id = grph[*vp.first].hypothesis_mv_[hyp_id].model_id_;
            Eigen::Matrix4f trans = grph[*vp.first].hypothesis_mv_[hyp_id].transform_;
            ModelTPtr model = grph[*vp.first].hypothesis_mv_[hyp_id].model_;

            std::stringstream name;
            name << cloud_name.str() << "_mv__hypothesis_" << hyp_id;

            // 		ModelTPtr m;

            // 		models_source_->getModelById(model_id, m);
            //
            // 		ConstPointInTPtr model_cloud = m->getAssembled (0.003f);
            typename pcl::PointCloud<PointT>::Ptr model_aligned ( new pcl::PointCloud<PointT> );
            //            if(model)
            //            {
            ConstPointInTPtr model_cloud = model->getAssembled (0.005f);
            pcl::transformPointCloud (*model_cloud, *model_aligned, trans);
            //            }
            //            else
            //            {
            //                typename pcl::PointCloud<PointT>::Ptr pModelPCl ( new pcl::PointCloud<PointT> );
            //                std::stringstream model_name;
            //                model_name << models_dir_ << model_id;
            //                pcl::io::loadPCDFile ( model_name.str(), *pModelPCl );
            //                pcl::transformPointCloud ( *pModelPCl, *model_aligned, trans );
            //            }

            pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb_handler3 ( model_aligned );
            vis->addPointCloud<PointT> ( model_aligned, rgb_handler3, name.str (), viewportNr[view_id * vis_rows + 3] );

            if ( grph[*vp.first].hypothesis_mv_[hyp_id].verified_ )	//--show-verified-extended-hypotheses
            {
                name << "__verified";
                pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb_handler4 ( model_aligned );
                vis->addPointCloud<PointT> ( model_aligned, rgb_handler4, name.str (), viewportNr[view_id * vis_rows + 4] );
            }
        }
    }
    vis->spin ();
    //vis->getInteractorStyle()->saveScreenshot ( "singleview.png" );
}

void multiviewGraph::
extendHypothesis ( Graph &grph )
{
    bool something_has_been_updated = true;
    std::pair<vertex_iter, vertex_iter> vp; //vp.first = running iterator..... vp.second = last iterator

    while ( something_has_been_updated )
    {
        something_has_been_updated = false;
        for ( vp = vertices ( grph ); vp.first != vp.second; ++vp.first )
        {
            typename graph_traits<Graph>::out_edge_iterator out_i, out_end;
            for ( tie ( out_i, out_end ) = out_edges ( *vp.first, grph ); out_i != out_end; ++out_i )
            {
                Edge e = *out_i;
                Vertex src = source ( e, grph ), targ = target ( e, grph );

                if ( grph[src].pScenePCl->header.frame_id != grph[*vp.first].pScenePCl->header.frame_id )
                    PCL_WARN("something's wrong");

                size_t hypothesis_length_before_extension = grph[src].hypothesis.size ();

                for ( std::vector<Hypothesis<PointT> >::iterator it_hypB = grph[targ].hypothesis.begin (); it_hypB != grph[targ].hypothesis.end (); ++it_hypB )
                {
                    bool hypotheses_from_view_exist = false;

                    //---check-if-hypotheses-from-updating-view-already-exist-in-current-view------------------------
                    for ( size_t id_hypA = 0; id_hypA < hypothesis_length_before_extension; id_hypA++ )
                    {
                        if ( grph[src].hypothesis[id_hypA].origin_ == it_hypB->origin_ )
                        {
                            hypotheses_from_view_exist = true;
                        }
                    }
                    if ( !hypotheses_from_view_exist )
                    {
                        Eigen::Matrix4f tf;
                        if ( grph[e].source_id.compare( grph[*vp.first].pScenePCl->header.frame_id ) == 0 )
                        {
                            tf = grph[e].transformation.inverse () * it_hypB->transform_;
                        }
                        else
                        {
                            tf = grph[e].transformation * it_hypB->transform_;
                        }

                        Hypothesis<PointT> ht_temp ( it_hypB->model_id_, tf, it_hypB->origin_, true );
                        grph[*vp.first].hypothesis.push_back ( ht_temp );
                        something_has_been_updated = true;
                    }
                }
            }
        }
    }
}

void multiviewGraph::
createEdgesFromHypothesisMatchOnline ( const Vertex new_vertex, Graph &grph, std::vector<Edge> &edges )
{
    vertex_iter vertexItA, vertexEndA;
    for (boost::tie(vertexItA, vertexEndA) = vertices(grph_); vertexItA != vertexEndA; ++vertexItA)
    {
        if ( grph[*vertexItA].pScenePCl->header.frame_id.compare( grph[new_vertex].pScenePCl->header.frame_id ) == 0 )
        {
            continue;
        }

        ROS_INFO("Checking vertex %s, which has %ld hypotheses.", grph[*vertexItA].pScenePCl->header.frame_id.c_str(), grph[*vertexItA].hypothesis.size());
        for ( std::vector<Hypothesis<PointT> >::iterator it_hypA = grph[*vertexItA].hypothesis.begin (); it_hypA != grph[*vertexItA].hypothesis.end (); ++it_hypA )
        {
            for ( std::vector<Hypothesis<PointT> >::iterator it_hypB = grph[new_vertex].hypothesis.begin (); it_hypB != grph[new_vertex].hypothesis.end (); ++it_hypB )
            {
                if ( it_hypB->model_id_.compare (it_hypA->model_id_ ) == 0 ) //model exists in other file (same id) --> create connection
                {
                    Eigen::Matrix4f tf_temp = it_hypB->transform_ * it_hypA->transform_.inverse (); //might be the other way around

                    //link views by an edge (for other graph)
                    Edge e_cpy;
                    bool b;
                    tie ( e_cpy, b ) = add_edge ( *vertexItA, new_vertex, grph );
                    grph[e_cpy].transformation = tf_temp;
                    grph[e_cpy].model_name = it_hypA->model_id_;
                    grph[e_cpy].source_id = grph[*vertexItA].pScenePCl->header.frame_id;
                    grph[e_cpy].target_id = grph[new_vertex].pScenePCl->header.frame_id;
                    grph[e_cpy].edge_weight = std::numeric_limits<double>::max ();
                    edges.push_back ( e_cpy );

                    std::cout << "Creating edge from view " << grph[*vertexItA].pScenePCl->header.frame_id << " to view " << grph[new_vertex].pScenePCl->header.frame_id
                              << " for model match " << grph[e_cpy].model_name
                              << std::endl;
                }
            }
        }
    }
}


/*
 * Correspondence grouping (clustering) for existing feature matches. If enough points (>cg_size) are
 * in the same cluster and vote for the same model, a hypothesis (with pose estimate) is constructed.
 */
void multiviewGraph::constructHypothesesFromFeatureMatches(std::map < std::string,faat_pcl::rec_3d_framework::ObjectHypothesis<PointT> > hypothesesInput,
                                                           pcl::PointCloud<PointT>::Ptr pKeypoints,
                                                           pcl::PointCloud<pcl::Normal>::Ptr pKeypointNormals,
                                                           std::vector<Hypothesis<PointT> > &hypothesesOutput,
                                                           std::vector <pcl::Correspondences>  &corresp_clusters_hyp)
{
    boost::shared_ptr < pcl::CorrespondenceGrouping<PointT, PointT> > cast_cg_alg;
    boost::shared_ptr < faat_pcl::GraphGeometricConsistencyGrouping<PointT, PointT> > gcg_alg (
                new faat_pcl::GraphGeometricConsistencyGrouping<
                PointT, PointT>);

    int cg_size = 7;
    gcg_alg->setGCThreshold (cg_size);
    gcg_alg->setGCSize (0.015);
    gcg_alg->setRansacThreshold (0.015);
    gcg_alg->setUseGraph (true);
    gcg_alg->setDistForClusterFactor (0);
    gcg_alg->setMaxTaken(2);
    gcg_alg->setMaxTimeForCliquesComputation(100);
    gcg_alg->setDotDistance (0.2);

    cast_cg_alg = boost::static_pointer_cast<pcl::CorrespondenceGrouping<PointT, PointT> > (gcg_alg);

    hypothesesOutput.clear();
    typename std::map<std::string, faat_pcl::rec_3d_framework::ObjectHypothesis<PointT> >::iterator it_map;
    std::cout << "I have " << hypothesesInput.size() << " hypotheses. " << std::endl;
    for (it_map = hypothesesInput.begin (); it_map != hypothesesInput.end (); it_map++)
    {
        if(it_map->second.correspondences_to_inputcloud->size() < 3)
            continue;

        std::vector <pcl::Correspondences> corresp_clusters;
        std::string id = it_map->second.model_->id_;
        std::cout << id << ": " << it_map->second.correspondences_to_inputcloud->size() << std::endl;
        cast_cg_alg->setSceneCloud (pKeypoints);
        cast_cg_alg->setInputCloud ((*it_map).second.correspondences_pointcloud);

        if(cast_cg_alg->getRequiresNormals())
        {
            //boost::shared_ptr< pcl::PointCloud<pcl::Normal> > pKeypointsSceneNormals;
            //pKeypointsSceneNormals.reset(new pcl::PointCloud<pcl::Normal>());
            //pcl::copyPointCloud(*(grph[vrtx].pSceneNormals_f_), grph[vrtx].keypointIndices_.indices, *pKeypointsSceneNormals);
            std::cout << "CG alg requires normals..." << ((*it_map).second.normals_pointcloud)->points.size() << " " << pKeypointNormals->points.size() << std::endl;
            assert(pKeypoints->points.size() == pKeypointNormals->points.size());
            cast_cg_alg->setInputAndSceneNormals((*it_map).second.normals_pointcloud, pKeypointNormals);
        }
        //we need to pass the keypoints_pointcloud and the specific object hypothesis
        cast_cg_alg->setModelSceneCorrespondences ((*it_map).second.correspondences_to_inputcloud);
        cast_cg_alg->cluster (corresp_clusters);

        std::cout << "Instances:" << corresp_clusters.size () << " Total correspondences:" << (*it_map).second.correspondences_to_inputcloud->size () << " " << it_map->first << std::endl;
        for (size_t i = 0; i < corresp_clusters.size (); i++)
        {
            //std::cout << "size cluster:" << corresp_clusters[i].size() << std::endl;
            Eigen::Matrix4f best_trans;
            typename pcl::registration::TransformationEstimationSVD < PointT, PointT > t_est;
            t_est.estimateRigidTransformation (*(*it_map).second.correspondences_pointcloud, *pKeypoints, corresp_clusters[i], best_trans);

            Hypothesis<PointT> ht_temp ( (*it_map).second.model_, best_trans);
            hypothesesOutput.push_back(ht_temp);
            corresp_clusters_hyp.push_back(corresp_clusters[i]);
            //        std::cout << "Model: " << (*it_map).second.model_->id_ << std::endl
            //                  << "Trans: " << best_trans << std::endl;
            //        models_->push_back ((*it_map).second.model_);
            //        transforms_->push_back (best_trans);
        }
    }
}

void multiviewGraph::
calcEdgeWeight (Graph &grph, std::vector<Edge> &edges)
{
//#pragma omp parallel for
    for (size_t i=0; i<edges.size(); i++) //std::vector<Edge>::iterator edge_it = edges.begin(); edge_it!=edges.end(); ++edge_it)
    {
        Edge edge = edges[i];

        const Vertex vrtx_src = source ( edge, grph );
        const Vertex vrtx_trgt = target ( edge, grph );

        Eigen::Matrix4f transform;
        if ( grph[edge].source_id.compare( grph[vrtx_src].pScenePCl->header.frame_id ) == 0)
        {
            transform = grph[edge].transformation;
        }
        else
        {
            transform = grph[edge].transformation.inverse ();
        }

        float w_after_icp_ = std::numeric_limits<float>::max ();
        const float best_overlap_ = 0.75f;

        Eigen::Matrix4f icp_trans;
        faat_pcl::registration::FastIterativeClosestPointWithGC<pcl::PointXYZRGB> icp;
        icp.setMaxCorrespondenceDistance ( 0.02f );
        icp.setInputSource (grph[vrtx_src].pScenePCl_f);
        icp.setInputTarget (grph[vrtx_trgt].pScenePCl_f);
        icp.setUseNormals (true);
        icp.useStandardCG (true);
        icp.setNoCG(true);
        icp.setOverlapPercentage (best_overlap_);
        icp.setKeepMaxHypotheses (5);
        icp.setMaximumIterations (10);
        icp.align (transform);
        w_after_icp_ = icp.getFinalTransformation ( icp_trans );

        if ( w_after_icp_ < 0 || !pcl_isfinite ( w_after_icp_ ) )
        {
            w_after_icp_ = std::numeric_limits<float>::max ();
        }
        else
        {
            w_after_icp_ = best_overlap_ - w_after_icp_;
        }

        if ( grph[edge].source_id.compare( grph[vrtx_src].pScenePCl->header.frame_id ) == 0)
        {
            PCL_WARN ( "Normal...\n" );
            //icp trans is aligning source to target
            //transform is aligning source to target
            //grph[edges[edge_id]].transformation = icp_trans * grph[edges[edge_id]].transformation;
            grph[edge].transformation = icp_trans;
        }
        else
        {
            //transform is aligning target to source
            //icp trans is aligning source to target
            PCL_WARN ( "Inverse...\n" );
            //grph[edges[edge_id]].transformation = icp_trans.inverse() * grph[edges[edge_id]].transformation;
            grph[edge].transformation = icp_trans.inverse ();
        }

        grph[edge].edge_weight = w_after_icp_;

        std::cout << "WEIGHT IS: " << grph[edge].edge_weight << " coming from edge connecting " << grph[edge].source_id
                  << " and " << grph[edge].target_id << " by object_id: " << grph[edge].model_name
                  << std::endl;
    }
}

Eigen::Matrix4f GeometryMsgToMatrix4f(const geometry_msgs::Transform & tt)
{
    Eigen::Matrix4f trans = Eigen::Matrix4f::Identity();
    trans(0,3) = tt.translation.x;
    trans(1,3) = tt.translation.y;
    trans(2,3) = tt.translation.z;

    Eigen::Quaternionf q(tt.rotation.w,tt.rotation.x,tt.rotation.y,tt.rotation.z);
    trans.block<3,3>(0,0) = q.toRotationMatrix();
    return trans;
}


/*
 * recognition passing a global transformation matrix (to a world coordinate system)
 */
bool multiviewGraph::recognize
(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr pInputCloud,
 std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > &hyp_transforms_local,
 std::vector<std::string> &hyp_model_ids,
 const std::string view_name,
 const size_t timestamp,
 const Eigen::Matrix4f global_transform)
{
    use_robot_pose_ = true;
    current_global_transform_ = global_transform;
    recognize(pInputCloud, hyp_transforms_local, hyp_model_ids, view_name, timestamp);
}


bool multiviewGraph::recognize
(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr pInputCloud,
 std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > &hyp_transforms_local,
 std::vector<std::string> &hyp_model_ids,
 const std::string view_name,
 const size_t timestamp)
{

    boost::shared_ptr< pcl::PointCloud<pcl::Normal> > pSceneNormals_f (new pcl::PointCloud<pcl::Normal> );

    assert(pInputCloud->width == 640 && pInputCloud->height == 480);
    Vertex vrtx = boost::add_vertex ( grph_ );

    pcl::copyPointCloud(*pInputCloud, *(grph_[vrtx].pScenePCl));
    //    grph_[vrtx].pScenePCl = pInputCloud;
    grph_[vrtx].pScenePCl->header.stamp = timestamp;
    grph_[vrtx].pScenePCl->header.frame_id = view_name;
    if(use_robot_pose_)
        grph_[vrtx].transform_to_world_co_system_ = current_global_transform_;

    if(chop_at_z_ > 0)
    {
        pcl::PassThrough<PointT> pass;
        pass.setFilterLimits (0.f, chop_at_z_);
        pass.setFilterFieldName ("z");
        pass.setInputCloud (grph_[vrtx].pScenePCl);
        pass.setKeepOrganized (true);
        pass.filter (*(grph_[vrtx].pScenePCl_f));
        grph_[vrtx].filteredSceneIndices_.indices = *(pass.getIndices());
        grph_[vrtx].filteredSceneIndices_.header.stamp = timestamp;
        grph_[vrtx].filteredSceneIndices_.header.frame_id = view_name;
    }

    //-----Normal estimation ---------------
    pcl::NormalEstimationOMP<PointT, pcl::Normal> ne;
    ne.setRadiusSearch ( 0.02f );
    ne.setInputCloud ( grph_[vrtx].pScenePCl );
    ne.compute ( *(grph_[vrtx].pSceneNormals) );

    pcl::copyPointCloud(*(grph_[vrtx].pSceneNormals), grph_[vrtx].filteredSceneIndices_, *pSceneNormals_f);
    assert(grph_[vrtx].pScenePCl_f->points.size() == pSceneNormals_f->points.size() );

    std::vector<Edge> new_edges;

    //--------------create-edges-between-views-by-Robot-Pose-----------------------------
    if(use_robot_pose_ && num_vertices(grph_)>1)
    {
        vertex_iter vertexIt, vertexEnd;
        for (boost::tie(vertexIt, vertexEnd) = vertices(grph_); vertexIt != vertexEnd; ++vertexIt)
        {
            Edge edge;
            if( grph_[*vertexIt].pScenePCl->header.frame_id.compare ( grph_[vrtx].pScenePCl->header.frame_id ) != 0 )
            {
                estimateViewTransformationByRobotPose ( *vertexIt, vrtx, grph_, edge );
                new_edges.push_back ( edge );
            }
        }
    }
    //----------END-create-edges-between-views-by-robot_pose-----------------------------------


    //----------create-edges-between-views-by-SIFT-----------------------------------
    if(scene_to_scene_)
    {
        calcSiftFeatures ( vrtx, grph_ );
        std::cout << "keypoints: " << grph_[vrtx].siftKeypointIndices_.indices.size() << std::endl;

        if (num_vertices(grph_)>1)
        {
            flann::Matrix<float> flann_data;
            flann::Index<DistT> *flann_index;
            multiview::convertToFLANN<pcl::Histogram<128> > ( grph_[vrtx].pSiftSignatures_, flann_data );
            flann_index = new flann::Index<DistT> ( flann_data, flann::KDTreeIndexParams ( 4 ) );
            flann_index->buildIndex ();

            //#pragma omp parallel for
            vertex_iter vertexIt, vertexEnd;
            for (boost::tie(vertexIt, vertexEnd) = vertices(grph_); vertexIt != vertexEnd; ++vertexIt)
            {
                Eigen::Matrix4f transformation;
                if( grph_[*vertexIt].pScenePCl->header.frame_id.compare ( grph_[vrtx].pScenePCl->header.frame_id ) != 0 )
                {
                    std::vector<Edge> edge;
                    estimateViewTransformationBySIFT ( *vertexIt, vrtx, grph_, flann_index, transformation, edge, use_gc_s2s_ );
                    for(size_t kk=0; kk < edge.size(); kk++)
                    {
                        new_edges.push_back (edge[kk]);
                    }
                }
            }
            delete flann_index;
        }
    }
    //----------END-create-edges-between-views-by-SIFT-----------------------------------


    //----------call-single-view-recognizer----------------------------------------------
    pSingleview_recognizer_->setISPK<flann::L1, pcl::Histogram<128> > (grph_[vrtx].pSiftSignatures_,
                                                                       grph_[vrtx].pScenePCl_f,
                                                                       grph_[vrtx].siftKeypointIndices_,
                                                                       faat_pcl::rec_3d_framework::SIFT);
    pSingleview_recognizer_->setInputCloud(grph_[vrtx].pScenePCl_f, pSceneNormals_f);
    pSingleview_recognizer_->constructHypotheses();
    pSingleview_recognizer_->getSavedHypotheses(grph_[vrtx].hypotheses_);
    pSingleview_recognizer_->getKeypointsMultipipe(grph_[vrtx].pKeypointsMultipipe_);
    pSingleview_recognizer_->getKeypointIndices(grph_[vrtx].keypointIndices_);
    pcl::copyPointCloud(*pSceneNormals_f, grph_[vrtx].keypointIndices_, *(grph_[vrtx].pKeypointNormalsMultipipe_));
    assert(grph_[vrtx].pKeypointNormalsMultipipe_->points.size()
           == grph_[vrtx].pKeypointsMultipipe_->points.size());

    //----display-keypoints--------------------
    //     pcl::visualization::PCLVisualizer::Ptr vis_temp (new pcl::visualization::PCLVisualizer);
    //     pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> handler_rgb_verified (grph[*it_vrtx].pScenePCl);
    //     vis_temp->addPointCloud<pcl::PointXYZRGB> (grph[*it_vrtx].pScenePCl, handler_rgb_verified, "Scene Cloud");
    //     pcl::PointCloud<pcl::PointNormal>::Ptr pKeypointWithNormal (new pcl::PointCloud<pcl::PointNormal>);
    //     for (size_t i=0; i < pKeypointNormals->points.size(); i++)
    //     {
    //         pcl::PointNormal ptNormal;
    //         ptNormal.getVector3fMap() = pKeypointNormals->at(i).getVector3fMap();
    //         ptNormal.x = grph[*it_vrtx].pKeypointsMultipipe_->at(i).x;
    //         ptNormal.y = grph[*it_vrtx].pKeypointsMultipipe_->at(i).y;
    //         ptNormal.z = grph[*it_vrtx].pKeypointsMultipipe_->at(i).z;
    //         pKeypointWithNormal ->points.push_back(ptNormal);
    //     }

    //     pcl::visualization::PointClou<pcl::PointXYZRGB> handler_rgb_verified2 (grph[*it_vrtx].pKeypointsMultipipe_);
    //     for (size_t keyId = 0; keyId < grph[*it_vrtx].pSiftKeypoints->size (); keyId++)
    //     {
    //         std::stringstream sphere_name;
    //         sphere_name << "sphere_" << keyId;
    //         vis_temp->addSphere<pcl::PointXYZRGB> (grph[*it_vrtx].pSiftKeypoints->at (keyId), 0.01, sphere_name.str ());
    //     }
    //     vis_temp->spin ();
//    std::cout << "Found " << grph_[vrtx].pKeypointsMultipipe_->points.size() << " multipipe keypoints." << std::endl;
    //    pcl::PointCloud<pcl::PointXYZ>::Ptr pKeypoints (new pcl::PointCloud<pcl::PointXYZ>);
    //    for (size_t i=0; i < grph_[vrtx].pKeypointsMultipipe_->points.size(); i++)
    //    {
    //         pcl::PointXYZ ptKeypoint;
    //         ptKeypoint.x = grph_[vrtx].pKeypointsMultipipe_->at(i).x;
    //         ptKeypoint.y = grph_[vrtx].pKeypointsMultipipe_->at(i).y;
    //         ptKeypoint.z = grph_[vrtx].pKeypointsMultipipe_->at(i).z;
    //         pKeypoints->points.push_back(ptKeypoint);
    //    }
    //    pcl::visualization::PCLVisualizer viewer("Keypoint Viewer");
    //    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> handler_rgb (grph_[vrtx].pScenePCl_f);
    //    viewer.addPointCloud<pcl::PointXYZRGB>(grph_[vrtx].pScenePCl_f, handler_rgb, "scene");
    //    viewer.addPointCloudNormals<pcl::PointXYZRGB,pcl::Normal>(grph_[vrtx].pKeypointsMultipipe_, pKeypointNormals, 10, 0.04);
    //    while (!viewer.wasStopped ())
    //    viewer.spin();


    std::vector < pcl::Correspondences > corresp_clusters_sv;
    constructHypothesesFromFeatureMatches(grph_[vrtx].hypotheses_,
                                          grph_[vrtx].pKeypointsMultipipe_,
                                          grph_[vrtx].pKeypointNormalsMultipipe_,
                                          grph_[vrtx].hypothesis,
                                          corresp_clusters_sv);
    //assert(grph_[vrtx].hypothesis.size() == corresp_clusters_sv.size());

    boost::shared_ptr<std::vector<ModelTPtr> > pModels_sv (new std::vector<ModelTPtr>);
    boost::shared_ptr<std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > > pTransforms_sv
            (new std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > );
    pTransforms_sv->resize (grph_[vrtx].hypothesis.size());
    pModels_sv->resize (grph_[vrtx].hypothesis.size());
    for(size_t hyp_id=0; hyp_id < grph_[vrtx].hypothesis.size(); hyp_id++)
    {
        pTransforms_sv->at(hyp_id) = grph_[vrtx].hypothesis[hyp_id].transform_;
        pModels_sv->at(hyp_id) = grph_[vrtx].hypothesis[hyp_id].model_;
    }
    pSingleview_recognizer_->poseRefinement(pModels_sv, pTransforms_sv);
    pSingleview_recognizer_->setModelsAndTransforms(*pModels_sv, *pTransforms_sv);
    std::vector<bool> mask_hv_sv;
    pSingleview_recognizer_->hypothesesVerificationGpu(mask_hv_sv);
    //    assert(mask_hv_sv.size() == pModels_sv->size());

    for (size_t j = 0; j < grph_[vrtx].hypothesis.size(); j++)
    {
        grph_[vrtx].hypothesis[j].verified_ = mask_hv_sv[j];
    }
    //----------END-call-single-view-recognizer------------------------------------------

    //createEdgesFromHypothesisMatchOnline ( vrtx, grph_, new_edges );
    calcEdgeWeight (grph_, new_edges);
    outputgraph ( grph_, "complete_graph.dot" );

    //---copy-vertices-to-graph_final----------------------------
    Vertex vrtx_final = boost::add_vertex ( grph_final_ );
    grph_final_[vrtx_final] = grph_[vrtx]; // shallow copy is okay here
    //    shallowCopyVertexIntoOtherGraph(vrtx, grph_, vrtx_final, grph_final_);

    //calcMST (edges_, grph, edges_final);      //No Minimum Spanning Tree calculation at the moment

    if ( new_edges.size() )
    {
        //------find best edge from the freshly inserted view and add it to the final graph-----------------
        Edge best_edge;
        best_edge = new_edges[0];

        for ( size_t i = 1; i < new_edges.size(); i++ )
        {
            //            visualizeEdge(new_edges[new_edge_id], grph_);
            //            edges_.push_back ( new_edges[new_edge_id] );

            if ( grph_[new_edges[i]].edge_weight < grph_[best_edge].edge_weight )
            {
                best_edge = new_edges[i];
            }
        }

        Vertex vrtx_src, vrtx_trgt;
        vrtx_src = source ( best_edge, grph_ );
        vrtx_trgt = target ( best_edge, grph_ );

        Edge e_cpy; bool b;
        tie ( e_cpy, b ) = add_edge ( vrtx_src, vrtx_trgt, grph_final_ );
        grph_final_[e_cpy] = grph_[best_edge]; // shallow copy is okay here
        //copyEdgeIntoOtherGraph(best_edge, grph_, e_cpy, grph_final_);
        //        best_edges_.push_back ( best_edge );
    }

    //---------Extend-hypotheses-from-other-view(s)------------------------------------------
    accumulatedHypotheses_.clear();
    extendFeatureMatchesRecursive(grph_final_, vrtx_final, accumulatedHypotheses_, pAccumulatedKeypoints_, pAccumulatedKeypointNormals_);
    std::vector < pcl::Correspondences > corresp_clusters_mv;
    constructHypothesesFromFeatureMatches(accumulatedHypotheses_,
                                          pAccumulatedKeypoints_,
                                          pAccumulatedKeypointNormals_,
                                          grph_final_[vrtx_final].hypothesis_mv_,
                                          corresp_clusters_mv);
    //assert(corresp_clusters_mv.size() == mv_hypotheses_.size());

    boost::shared_ptr<std::vector<ModelTPtr> > pModels_mv (new std::vector<ModelTPtr> );
    boost::shared_ptr<std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > > pTransforms_mv
            (new std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > );
    pTransforms_mv->resize (grph_final_[vrtx_final].hypothesis_mv_.size());
    pModels_mv->resize (grph_final_[vrtx_final].hypothesis_mv_.size());
    for(size_t hyp_id=0; hyp_id < grph_final_[vrtx_final].hypothesis_mv_.size(); hyp_id++)
    {
        pTransforms_mv->at(hyp_id) = grph_final_[vrtx_final].hypothesis_mv_[hyp_id].transform_;
        pModels_mv->at(hyp_id) = grph_final_[vrtx_final].hypothesis_mv_[hyp_id].model_;
        assert(pModels_mv->at(hyp_id)->assembled_);
    }
    pSingleview_recognizer_->poseRefinement(pModels_mv, pTransforms_mv);
    pSingleview_recognizer_->setModelsAndTransforms(*pModels_mv, *pTransforms_mv);
    std::vector<bool> mask_hv_mv;
    pSingleview_recognizer_->hypothesesVerificationGpu(mask_hv_mv);

    for(size_t i=0; i<mask_hv_mv.size(); i++)
    {
        grph_final_[vrtx_final].hypothesis_mv_[i].verified_ = static_cast<int>(mask_hv_mv[i]);

        if(mask_hv_mv[i])
        {
            std::string id = grph_final_[vrtx_final].hypothesis_mv_[i].model_->id_;
            std::map<std::string, faat_pcl::rec_3d_framework::ObjectHypothesis<PointT> >::iterator it_hyp_sv;
            it_hyp_sv = grph_final_[vrtx_final].hypotheses_.find(id);
            if (it_hyp_sv == grph_final_[vrtx_final].hypotheses_.end())
            {
                std::cout << "There has not been a single keypoint detected for model " << id << std::endl;
            }
            else
            {
                std::cout << "Augmenting hypotheses with verified feature matches." << std::endl;
                for(size_t jj = 0; jj < corresp_clusters_mv[i].size(); jj++)
                {
                    pcl::Correspondence c = corresp_clusters_mv[i][jj];
                    int kp_scene_idx = c.index_match;
                    int kp_model_idx = c.index_query;
                    PointT keypoint_model = accumulatedHypotheses_[id].correspondences_pointcloud->points[kp_model_idx];
                    pcl::Normal keypoint_normal_model = accumulatedHypotheses_[id].normals_pointcloud->points[kp_model_idx];
                    PointT keypoint_scene = pAccumulatedKeypoints_->points[kp_scene_idx];
                    pcl::Normal keypoint_normal_scene = pAccumulatedKeypointNormals_->points[kp_scene_idx];

                    // only add correspondences which correspondences to both, model and scene keypoint,
                    // are not already saved in the hypotheses coming from single view recognition only.
                    // As the keypoints from single view rec. are pushed in the front, we only have to check
                    // if the indices of the new correspondences are outside of these keypoint sizes.
                    // Also, we don't have to check if these new keypoints are redundant because this
                    // is already done in the function "extendFeatureMatchesRecursive(..)".

//                    if(kp_model_idx >= it_hyp_sv->second.correspondences_pointcloud->points.size()
//                            && kp_scene_idx >= grph_final_[vrtx_final].pKeypointsMultipipe_->points.size())
//                    {
//                        std::cout << "THIS WOULD BE A CORRESPONDENCE TO AUGMENT. " << std::endl;
//                        it_hyp_sv->second.correspondences_pointcloud->points.push_back(keypoint_model);
//                        it_hyp_sv->second.normals_pointcloud->points.push_back(keypoint_normal_model);
//                        grph_final_[vrtx_final].pKeypointsMultipipe_->points.push_back(keypoint_scene);
//                        grph_final_[vrtx_final].pKeypointNormalsMultipipe_->points.push_back(keypoint_normal_scene);

//                        pcl::Correspondence c_new;
//                        c_new.index_match = grph_final_[vrtx_final].pKeypointsMultipipe_->points.size()-1;
//                        c_new.index_query = it_hyp_sv->second.correspondences_pointcloud->points.size()-1;
//                        it_hyp_sv->second.correspondences_to_inputcloud->push_back(c_new);
//                        it_hyp_sv->second.num_corr_++;
//                        // Note indices to flann and feature distance not implemented as we don't need it for
//                        // the current algorithm.
//                    }
                }
            }
        }
    }

    visualizeGraph(grph_final_, vis_);

    grph_[vrtx].cumulative_weight_to_new_vrtx_ = grph_final_[vrtx_final].cumulative_weight_to_new_vrtx_;

    std::pair<vertex_iter, vertex_iter> vp;
    for ( vp = vertices ( grph_final_ ); vp.first != vp.second; ++vp.first )
    {   //--reset-hop-status
        grph_final_ [*vp.first].has_been_hopped_ = false;
    }

    outputgraph ( grph_final_, "Final_with_Hypothesis_extension.dot" );
    //    visualizeGraph(grph_final_, vis_);

    //----respond-service-call-and-publish-all-recognized-models-(after-multiview-extension)-as-ROS-point-cloud-------------------
    //    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pRecognizedModels (new pcl::PointCloud<pcl::PointXYZRGB>);
    for ( size_t hyp_id = 0; hyp_id < grph_final_[vrtx_final].hypothesis.size(); hyp_id++ )
    {
        if ( grph_final_[vrtx_final].hypothesis[hyp_id].verified_ )
        {
            hyp_transforms_local.push_back(grph_final_[vrtx_final].hypothesis[hyp_id].transform_);
            hyp_model_ids.push_back(grph_final_[vrtx_final].hypothesis[hyp_id].model_id_);

            //            std_msgs::String model_id;
            //            model_id.data = grph_final_[vrtx_final].hypothesis[hyp_id].model_id_;
            //            response.ids.push_back(model_id);

            //            Eigen::Matrix4f trans = grph_final_[vrtx_final].hypothesis[hyp_id].transform_;
            //            geometry_msgs::Transform tt;
            //            tt.translation.x = trans(0,3);
            //            tt.translation.y = trans(1,3);
            //            tt.translation.z = trans(2,3);

            //            Eigen::Matrix3f rotation = trans.block<3,3>(0,0);
            //            Eigen::Quaternionf q(rotation);
            //            tt.rotation.x = q.x();
            //            tt.rotation.y = q.y();
            //            tt.rotation.z = q.z();
            //            tt.rotation.w = q.w();
            //            response.transforms.push_back(tt);

            //            Eigen::Matrix4f trans_2_world = grph_final_[vrtx_final].transform_to_world_co_system_ * trans;


            //            typename pcl::PointCloud<PointT>::Ptr pModelPCl ( new pcl::PointCloud<PointT> );
            //            typename pcl::PointCloud<PointT>::Ptr model_aligned ( new pcl::PointCloud<PointT> );
            //            pcl::io::loadPCDFile ( grph_final_[vrtx_final].hypothesis[hyp_id].model_id_, *pModelPCl );
            //            pcl::transformPointCloud ( *pModelPCl, *model_aligned, trans_2_world );
            //            *pRecognizedModels += *model_aligned;
        }
    }

    //-------Clean-up-graph-------------------
    pruneGraph(grph_, 2);
    pruneGraph(grph_final_, 2);

    outputgraph ( grph_final_, "final_after_deleting_old_vertex.dot" );
    outputgraph ( grph_, "grph_after_deleting_old_vertex.dot" );

    return true;
}
