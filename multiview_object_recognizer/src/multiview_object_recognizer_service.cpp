#define EIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET

#include "multiview_object_recognizer_service.h"
#include "geometry_msgs/Point32.h"
#include "geometry_msgs/Transform.h"
#include "std_msgs/Time.h"

bool multiviewGraph::visualize_output() const
{
    return visualize_output_;
}

void multiviewGraph::setVisualize_output(bool visualize_output)
{
    visualize_output_ = visualize_output;
}

double multiviewGraph::chop_at_z() const
{
    return chop_at_z_;
}

void multiviewGraph::setChop_at_z(double chop_at_z)
{
    chop_at_z_ = chop_at_z;
}

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
multiview::convertToFLANN ( const typename pcl::PointCloud<Type>::ConstPtr & cloud, flann::Matrix<float> &data )
{
    data.rows = cloud->points.size ();
    data.cols = sizeof ( cloud->points[0].histogram ) / sizeof ( float ); // number of histogram bins

    flann::Matrix<float> flann_data ( new float[data.rows * data.cols], data.rows, data.cols );

    for ( size_t i = 0; i < data.rows; ++i )
        for ( size_t j = 0; j < data.cols; ++j )
        {
            flann_data.ptr () [i * data.cols + j] = cloud->points[i].histogram[j];
        }

    data = flann_data;
}

template void
multiview::convertToFLANN<pcl::Histogram<128> > (const pcl::PointCloud<pcl::Histogram<128> >::ConstPtr & cloud, flann::Matrix<float> &data ); // explicit instantiation.


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
    temp_correspondences->resize(pSiftKeypointsSrc->size ());

    for ( size_t keypointId = 0; keypointId < pSiftKeypointsSrc->size (); keypointId++ )
    {
        FeatureT searchFeature = grph[src].pSiftSignatures_->at ( keypointId );
        int size_feat = sizeof ( searchFeature.histogram ) / sizeof ( float );
        multiview::nearestKSearch ( flann_index, searchFeature.histogram, size_feat, K, indices, distances );

        pcl::Correspondence corr;
        corr.distance = distances[0][0];
        corr.index_query = keypointId;
        corr.index_match = indices[0][0];
        temp_correspondences->at(keypointId) = corr;
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
    pcl::copyPointCloud(*(grph[vrtx_start].pKeypointsMultipipe_), *pKeypoints);
    pcl::copyPointCloud(*(grph[vrtx_start].pKeypointNormalsMultipipe_), *pKeypointNormals);

    std::map<std::string, faat_pcl::rec_3d_framework::ObjectHypothesis<PointT> >::const_iterator it_copy_hyp;
    for(it_copy_hyp = grph[vrtx_start].hypotheses_.begin(); it_copy_hyp !=grph[vrtx_start].hypotheses_.end(); ++it_copy_hyp)
    {
        faat_pcl::rec_3d_framework::ObjectHypothesis<PointT> oh;
        oh.model_ = it_copy_hyp->second.model_;
        oh.correspondences_pointcloud.reset(new pcl::PointCloud<PointT>(*(it_copy_hyp->second.correspondences_pointcloud)));
        oh.normals_pointcloud.reset(new pcl::PointCloud<pcl::Normal>(*(it_copy_hyp->second.normals_pointcloud)));
        oh.correspondences_to_inputcloud.reset(new pcl::Correspondences);
        *(oh.correspondences_to_inputcloud) = *(it_copy_hyp->second.correspondences_to_inputcloud);
        oh.indices_to_flann_models_ = it_copy_hyp->second.indices_to_flann_models_;
        hypotheses.insert(std::pair<std::string, faat_pcl::rec_3d_framework::ObjectHypothesis<PointT> >(it_copy_hyp->first, oh));
    }

    assert(pKeypoints->points.size() == pKeypointNormals->points.size());

    grph[vrtx_start].has_been_hopped_ = true;

    typename graph_traits<Graph>::out_edge_iterator out_i, out_end;
    tie ( out_i, out_end ) = out_edges ( vrtx_start, grph);
    if(out_i != out_end)  //otherwise there are no edges to hop
    {   //------get hypotheses from next vertex. Just taking the first one not being hopped.----
        std::string edge_src, edge_trgt;
        Vertex remote_vertex;
        Eigen::Matrix4f edge_transform;

        bool have_found_a_valid_edge = false;
        for (; out_i != out_end; ++out_i )
        {
            remote_vertex  = target ( *out_i, grph );
            edge_src       = grph[*out_i].source_id;
            edge_trgt      = grph[*out_i].target_id;
            edge_transform = grph[*out_i].transformation;

            if (! grph[remote_vertex].has_been_hopped_)
            {
                have_found_a_valid_edge = true;
                std::cout << "Edge src: " << edge_src << "; target: " << edge_trgt << "; model_name: " << grph[*out_i].model_name
                          << "; edge_weight: " << grph[*out_i].edge_weight << std::endl;
                break;
            }
        }
        if(have_found_a_valid_edge)
        {
            ROS_INFO("Hopping to vertex %s...", grph[remote_vertex].pScenePCl->header.frame_id.c_str());

            std::map<std::string, faat_pcl::rec_3d_framework::ObjectHypothesis<PointT> > new_hypotheses;
            pcl::PointCloud<PointT>::Ptr pNewKeypoints (new pcl::PointCloud<PointT>);
            pcl::PointCloud<pcl::Normal>::Ptr pNewKeypointNormals (new pcl::PointCloud<pcl::Normal>);
            extendFeatureMatchesRecursive ( grph, remote_vertex, new_hypotheses, pNewKeypoints, pNewKeypointNormals);
            assert( pNewKeypoints->size() == pNewKeypointNormals->size() );

            //------check for right transformation between the two vertices (edges are undirected, so we have to check the source/target attribute of the edge)------
            //------then transform new keypoints and keypoint normals to current vertex (vrtx_start)-----------
            Eigen::Matrix4f transform;
            if ( edge_src.compare( grph[vrtx_start].pScenePCl->header.frame_id ) == 0)
            {
                transform = edge_transform.inverse();
            }
            else if (edge_trgt.compare( grph[vrtx_start].pScenePCl->header.frame_id ) == 0)
            {
                transform = edge_transform;
            }
            else
            {
                PCL_ERROR("Something is messed up with the transformation.");
            }

            //------ Transform keypoints and rotate normals----------
            const Eigen::Matrix3f rot   = transform.block<3, 3> (0, 0);
            const Eigen::Vector3f trans = transform.block<3, 1> (0, 3);
            for (size_t i=0; i<pNewKeypoints->size(); i++)
            {
                pNewKeypoints->points[i].getVector3fMap() = rot * pNewKeypoints->points[i].getVector3fMap () + trans;
                pNewKeypointNormals->points[i].getNormalVector3fMap() = rot * pNewKeypointNormals->points[i].getNormalVector3fMap ();
            }

            // add/merge hypotheses
            pcl::PointCloud<pcl::PointXYZ>::Ptr pKeypointsXYZ (new pcl::PointCloud<pcl::PointXYZ>);
            pKeypointsXYZ->points.resize(pKeypoints->size());
            for(size_t i=0; i < pKeypoints->size(); i++)
            {
                pKeypointsXYZ->points[i].getVector3fMap() = pKeypoints->points[i].getVector3fMap();
            }

            std::map<std::string, faat_pcl::rec_3d_framework::ObjectHypothesis<PointT> >::const_iterator it_new_hyp;
            for(it_new_hyp = new_hypotheses.begin(); it_new_hyp !=new_hypotheses.end(); ++it_new_hyp)
            {
                const std::string id = it_new_hyp->second.model_->id_;

                std::map<std::string, faat_pcl::rec_3d_framework::ObjectHypothesis<PointT> >::iterator it_existing_hyp;
                it_existing_hyp = hypotheses.find(id);
                if (it_existing_hyp == hypotheses.end())
                {
                    PCL_ERROR("There are no hypotheses (feature matches) for this model yet.");
                    faat_pcl::rec_3d_framework::ObjectHypothesis<PointT> oh;
                    oh.correspondences_pointcloud.reset(new pcl::PointCloud<PointT>
                                                        (*(it_new_hyp->second.correspondences_pointcloud)));
                    oh.normals_pointcloud.reset(new pcl::PointCloud<pcl::Normal>
                                                (*(it_new_hyp->second.normals_pointcloud)));
                    oh.indices_to_flann_models_ = it_new_hyp->second.indices_to_flann_models_;
                    oh.correspondences_to_inputcloud.reset (new pcl::Correspondences);
                    *(oh.correspondences_to_inputcloud) = *(it_new_hyp->second.correspondences_to_inputcloud);
                    for(size_t i=0; i < oh.correspondences_to_inputcloud->size(); i++)
                    {
                        oh.correspondences_to_inputcloud->at(i).index_match += pKeypoints->points.size();
                    }
                    hypotheses.insert(std::pair<std::string, faat_pcl::rec_3d_framework::ObjectHypothesis<PointT> >(id, oh));
                }
                else
                { //merge hypotheses
                    for(size_t j=0; j < it_new_hyp->second.correspondences_to_inputcloud->size(); j++)
                    {
                        bool drop_new_correspondence = false;

                        pcl::Correspondence c_new = it_new_hyp->second.correspondences_to_inputcloud->at(j);
                        const PointT new_kp = pNewKeypoints->points[c_new.index_match];
                        pcl::PointXYZ new_kp_XYZ;
                        new_kp_XYZ.getVector3fMap() = new_kp.getVector3fMap();
                        const pcl::Normal new_kp_normal = pNewKeypointNormals->points[c_new.index_match];

                        const PointT new_model_pt = it_new_hyp->second.correspondences_pointcloud->points[ c_new.index_query ];
                        pcl::PointXYZ new_model_pt_XYZ;
                        new_model_pt_XYZ.getVector3fMap() = new_model_pt.getVector3fMap();
                        const pcl::Normal new_model_normal = it_new_hyp->second.normals_pointcloud->points[ c_new.index_query ];

                        if (!pcl::isFinite(new_kp_XYZ) || !pcl::isFinite(new_model_pt_XYZ))
                        {
                            PCL_WARN("Keypoint of scene or model is infinity!!");
                            continue;
                        }
                        for(size_t j=0; j < it_existing_hyp->second.correspondences_to_inputcloud->size(); j++)
                        {
                            const pcl::Correspondence c_existing = it_existing_hyp->second.correspondences_to_inputcloud->at(j);
                            const PointT existing_model_pt = it_existing_hyp->second.correspondences_pointcloud->points[ c_existing.index_query ];
                            pcl::PointXYZ existing_model_pt_XYZ;
                            existing_model_pt_XYZ.getVector3fMap() = existing_model_pt.getVector3fMap();

                            const PointT existing_kp = pKeypoints->points[c_existing.index_match];
                            pcl::PointXYZ existing_kp_XYZ;
                            existing_kp_XYZ.getVector3fMap() = existing_kp.getVector3fMap();
                            const pcl::Normal existing_kp_normal = pKeypointNormals->points[c_existing.index_match];

                            float squaredDistModelKeypoints = pcl::squaredEuclideanDistance(new_model_pt_XYZ, existing_model_pt_XYZ);
                            float squaredDistSceneKeypoints = pcl::squaredEuclideanDistance(new_kp_XYZ, existing_kp_XYZ);

                            if((squaredDistSceneKeypoints < distance_keypoints_get_discarded_) &&
                                    (new_kp_normal.getNormalVector3fMap().dot(existing_kp_normal.getNormalVector3fMap()) > 0.8) &&
                                    (squaredDistModelKeypoints < distance_keypoints_get_discarded_))
                            {
                                //                                std::cout << "Found a very close point (keypoint distance: " << squaredDistSceneKeypoints
                                //                                          << "; model distance: " << squaredDistModelKeypoints
                                //                                          << ") with the same model id and similar normal (Normal dot product: "
                                //                                          << new_kp_normal.getNormalVector3fMap().dot(existing_kp_normal.getNormalVector3fMap()) << "> 0.8). Ignoring it."
                                //                                                                      << std::endl;
                                drop_new_correspondence = true;
                                break;
                            }
                        }
                        if (!drop_new_correspondence)
                        {
                            it_existing_hyp->second.indices_to_flann_models_.push_back(
                                        it_new_hyp->second.indices_to_flann_models_[c_new.index_query]); //check that
                            c_new.index_query = it_existing_hyp->second.correspondences_pointcloud->points.size();
                            c_new.index_match += pKeypoints->points.size();
                            it_existing_hyp->second.correspondences_to_inputcloud->push_back(c_new);
                            it_existing_hyp->second.correspondences_pointcloud->points.push_back(new_model_pt);
                            it_existing_hyp->second.normals_pointcloud->points.push_back(new_model_normal);
                        }
                    }
                    //                    std::cout << "INFO: Size for " << id <<
                    //                                 " of correspondes_pointcloud after merge: " << it_existing_hyp->second.correspondences_pointcloud->points.size() << std::endl;
                }
            }
            *pKeypoints += *pNewKeypoints;
            *pKeypointNormals += *pNewKeypointNormals;
        }
    }
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

    for ( std::vector<Hypothesis<PointT>>::const_iterator it_hyp = grph[current_vertex].hypothesis_sv_.begin (); it_hyp != grph[current_vertex].hypothesis_sv_.end (); ++it_hyp )
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

                size_t hypothesis_length_before_extension = grph[src].hypothesis_sv_.size ();

                for ( std::vector<Hypothesis<PointT> >::iterator it_hypB = grph[targ].hypothesis_sv_.begin (); it_hypB != grph[targ].hypothesis_sv_.end (); ++it_hypB )
                {
                    bool hypotheses_from_view_exist = false;

                    //---check-if-hypotheses-from-updating-view-already-exist-in-current-view------------------------
                    for ( size_t id_hypA = 0; id_hypA < hypothesis_length_before_extension; id_hypA++ )
                    {
                        if ( grph[src].hypothesis_sv_[id_hypA].origin_ == it_hypB->origin_ )
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
                        grph[*vp.first].hypothesis_sv_.push_back ( ht_temp );
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

        ROS_INFO("Checking vertex %s, which has %ld hypotheses.", grph[*vertexItA].pScenePCl->header.frame_id.c_str(), grph[*vertexItA].hypothesis_sv_.size());
        for ( std::vector<Hypothesis<PointT> >::iterator it_hypA = grph[*vertexItA].hypothesis_sv_.begin (); it_hypA != grph[*vertexItA].hypothesis_sv_.end (); ++it_hypA )
        {
            for ( std::vector<Hypothesis<PointT> >::iterator it_hypB = grph[new_vertex].hypothesis_sv_.begin (); it_hypB != grph[new_vertex].hypothesis_sv_.end (); ++it_hypB )
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

void multiviewGraph::
calcEdgeWeight (Graph &grph, std::vector<Edge> &edges)
{
#pragma omp parallel for
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
 const std::string view_name,
 const size_t timestamp,
 const Eigen::Matrix4f global_transform)
{
    use_robot_pose_ = true;
    current_global_transform_ = global_transform;
    recognize(pInputCloud, view_name, timestamp);
}


bool multiviewGraph::recognize
(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr pInputCloud,
 const std::string view_name,
 const size_t timestamp)
{
    std::cout << "=================================================================" << std::endl <<
                 "Started recognition for view " << view_name << " in scene " << scene_name_ << " with timestamp " << timestamp <<
                 " and following settings for mv_recognizer: " << std::endl <<
                 "***Max vertices kept in graph: " << max_vertices_in_graph_ << std::endl <<
                 "***euclidean distance keypoints get discarded during extension: " << distance_keypoints_get_discarded_ << std::endl <<
                 "***chop at z: " << chop_at_z_ << std::endl <<
                 "***scene to scene calculation: " << scene_to_scene_ << std::endl <<
                 "***robot pose for transform calculation: " << use_robot_pose_ << std::endl <<
                 "=========================================================" << std::endl << std::endl;

    double  total_time,
            robot_pose_est_time,
            sift_feate_est_time,
            sv_hyp_construction_time,
            sv_overhead_time,
            mv_hyp_construct_time,
            mv_hyp_ver_time,
            mv_feat_ext_time,
            mv_icp_time,
            sv_total_time;

    times_.clear();

    pcl::ScopeTime total_pcl_time ("Multiview Recognition");

    boost::shared_ptr< pcl::PointCloud<pcl::Normal> > pSceneNormals_f (new pcl::PointCloud<pcl::Normal> );

    assert(pInputCloud->width == 640 && pInputCloud->height == 480);
    Vertex vrtx = boost::add_vertex ( grph_ );

    pcl::copyPointCloud(*pInputCloud, *(grph_[vrtx].pScenePCl));
    //    grph_[vrtx].pScenePCl = pInputCloud;
    grph_[vrtx].pScenePCl->header.stamp = timestamp;
    grph_[vrtx].pScenePCl->header.frame_id = view_name;
    most_current_view_id_ = view_name;

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


    //----------calc-SIFT-features-create-edges-between-views-by-SIFT-----------------------------------
    if(scene_to_scene_)
    {
        pcl::ScopeTime ticp ("SIFT scene to scene registration...");
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
    if(scene_to_scene_)
    {
        pSingleview_recognizer_->setISPK<flann::L1, pcl::Histogram<128> > (grph_[vrtx].pSiftSignatures_,
                                                                           grph_[vrtx].pScenePCl_f,
                                                                           grph_[vrtx].siftKeypointIndices_,
                                                                           faat_pcl::rec_3d_framework::SIFT);
    }
    pSingleview_recognizer_->setInputCloud(grph_[vrtx].pScenePCl_f, pSceneNormals_f);
    pSingleview_recognizer_->constructHypotheses();
    pSingleview_recognizer_->getSavedHypotheses(grph_[vrtx].hypotheses_);
    pSingleview_recognizer_->getKeypointsMultipipe(grph_[vrtx].pKeypointsMultipipe_);
    pSingleview_recognizer_->getKeypointIndices(grph_[vrtx].keypointIndices_);
    pcl::copyPointCloud(*pSceneNormals_f, grph_[vrtx].keypointIndices_, *(grph_[vrtx].pKeypointNormalsMultipipe_));
    assert(grph_[vrtx].pKeypointNormalsMultipipe_->points.size()
           == grph_[vrtx].pKeypointsMultipipe_->points.size());

    size_t sv_num_correspondences=0;
    std::map<std::string, faat_pcl::rec_3d_framework::ObjectHypothesis<PointT> >::const_iterator it_hyp;
    for(it_hyp = grph_[vrtx].hypotheses_.begin(); it_hyp !=grph_[vrtx].hypotheses_.end(); ++it_hyp)
    {
        sv_num_correspondences += it_hyp->second.correspondences_to_inputcloud->size();
    }

    pcl::StopWatch sv_overhead_pcl_time;

    std::vector < pcl::Correspondences > corresp_clusters_sv;

    {
        pcl::ScopeTime sv_hyp_construction ("Constructing hypotheses from feature matches...");
        pSingleview_recognizer_->constructHypothesesFromFeatureMatches(grph_[vrtx].hypotheses_,
                                                                       grph_[vrtx].pKeypointsMultipipe_,
                                                                       grph_[vrtx].pKeypointNormalsMultipipe_,
                                                                       grph_[vrtx].hypothesis_sv_,
                                                                       corresp_clusters_sv);
        sv_hyp_construction_time = sv_hyp_construction.getTime();
    }
    //    std::vector<float> fsv_mask_sv;
    //    pSingleview_recognizer_->preFilterWithFSV(grph_[vrtx].pScenePCl, fsv_mask_sv);
    //        pcl::visualization::PCLVisualizer::Ptr vis_fsv (new pcl::visualization::PCLVisualizer);
    //        for(size_t i=0; i<fsv_mask_sv.size(); i++)
    //        {
    //            vis_fsv->removeAllPointClouds();
    //            pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> handler_rgb_verified (grph_[vrtx].pScenePCl);
    //            vis_fsv->addPointCloud<pcl::PointXYZRGB> (grph_[vrtx].pScenePCl, handler_rgb_verified, "scene_cloud");
    //            std::cout << "fsv " << i << ": " << fsv_mask_sv[i] << std::endl;
    //            std::stringstream model_id;
    //            model_id << "cloud " << i;
    //            ConstPointInTPtr model_cloud = grph_[vrtx].hypothesis_sv_[i].model_->getAssembled (0.005f);
    //            typename pcl::PointCloud<PointT>::Ptr model_aligned (new pcl::PointCloud<PointT>);
    //            pcl::transformPointCloud (*model_cloud, *model_aligned, grph_[vrtx].hypothesis_sv_[i].transform_);
    //            pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> handler_rgb_verified2 (model_aligned);
    //            vis_fsv->addPointCloud<pcl::PointXYZRGB> (model_aligned, handler_rgb_verified2, model_id.str());
    //            vis_fsv->spin();
    //        }

    pSingleview_recognizer_->poseRefinement();
    std::vector<bool> mask_hv_sv;
    {
        pcl::ScopeTime ticp ("Hypotheses verification...");
        pSingleview_recognizer_->hypothesesVerification(mask_hv_sv);
        pSingleview_recognizer_->getVerifiedPlanes(grph_[vrtx].verified_planes_);
    }
    for (size_t j = 0; j < grph_[vrtx].hypothesis_sv_.size(); j++)
    {
        grph_[vrtx].hypothesis_sv_[j].verified_ = mask_hv_sv[j];
    }

    sv_overhead_time = sv_overhead_pcl_time.getTime();
    sv_total_time = total_pcl_time.getTime();
    //----------END-call-single-view-recognizer------------------------------------------

    //---copy-vertices-to-graph_final----------------------------
    Vertex vrtx_final = boost::add_vertex ( grph_final_ );
    grph_final_[vrtx_final] = grph_[vrtx]; // shallow copy is okay here

    if(new_edges.size())
    {
        Edge best_edge;
        best_edge = new_edges[0];

        if(new_edges.size()>1)  // take the "best" edge for transformation between the views
        {
            pcl::ScopeTime ticp ("Calculating edge weights...");
            calcEdgeWeight (grph_, new_edges);

            //------find best edge from the freshly inserted view and add it to the final graph-----------------

            for ( size_t i = 1; i < new_edges.size(); i++ )
            {
                //                bgvis_.visualizeEdge(new_edges[i], grph_);

                if ( grph_[new_edges[i]].edge_weight < grph_[best_edge].edge_weight )
                {
                    best_edge = new_edges[i];
                }
            }
        }
        //        bgvis_.visualizeEdge(new_edges[0], grph_);
        Vertex vrtx_src, vrtx_trgt;
        vrtx_src = source ( best_edge, grph_ );
        vrtx_trgt = target ( best_edge, grph_ );

        Edge e_cpy; bool b;
        tie ( e_cpy, b ) = add_edge ( vrtx_src, vrtx_trgt, grph_final_ );
        grph_final_[e_cpy] = grph_[best_edge]; // shallow copy is okay here
    }
    else
    {
        std::cout << "No edge for this vertex." << std::endl;
    }

    //---------Extend-hypotheses-from-other-view(s)------------------------------------------
    accumulatedHypotheses_.clear();

    pcl::StopWatch mv_feat_ext_pcl_time;
    extendFeatureMatchesRecursive(grph_final_, vrtx_final, accumulatedHypotheses_, pAccumulatedKeypoints_, pAccumulatedKeypointNormals_);

    size_t total_num_correspondences=0;
    for(it_hyp = accumulatedHypotheses_.begin(); it_hyp != accumulatedHypotheses_.end(); ++it_hyp)
    {
        total_num_correspondences += it_hyp->second.correspondences_to_inputcloud->size();
    }

    mv_feat_ext_time = mv_feat_ext_pcl_time.getTime();

    std::vector < pcl::Correspondences > corresp_clusters_mv;

    pcl::StopWatch mv_hyp_construct;
    pSingleview_recognizer_->constructHypothesesFromFeatureMatches(accumulatedHypotheses_,
                                                                   pAccumulatedKeypoints_,
                                                                   pAccumulatedKeypointNormals_,
                                                                   grph_final_[vrtx_final].hypothesis_mv_,
                                                                   corresp_clusters_mv);
    mv_hyp_construct_time = mv_hyp_construct.getTime();

    {
        pcl::ScopeTime ticp("Multi-view ICP...");
        pSingleview_recognizer_->poseRefinement();
        mv_icp_time = ticp.getTime();
    }
    std::vector<bool> mask_hv_mv;

    pcl::StopWatch mv_hyp_ver_pcl_time;
    pSingleview_recognizer_->hypothesesVerification(mask_hv_mv);
    mv_hyp_ver_time = mv_hyp_ver_pcl_time.getTime();


    for(size_t i=0; i<mask_hv_mv.size(); i++)
    {
        grph_final_[vrtx_final].hypothesis_mv_[i].verified_ = static_cast<int>(mask_hv_mv[i]);

        if(mask_hv_mv[i])
        {
            const std::string id = grph_final_[vrtx_final].hypothesis_mv_[i].model_->id_;
            std::map<std::string, faat_pcl::rec_3d_framework::ObjectHypothesis<PointT> >::iterator it_hyp_sv;
            it_hyp_sv = grph_final_[vrtx_final].hypotheses_.find(id);
            if (it_hyp_sv == grph_final_[vrtx_final].hypotheses_.end())
            {
                PCL_ERROR("There has not been a single keypoint detected for model %s", id.c_str());
            }
            else
            {
                std::cout << "Augmenting hypotheses with verified feature matches." << std::endl;
                for(size_t jj = 0; jj < corresp_clusters_mv[i].size(); jj++)
                {
                    const pcl::Correspondence c = corresp_clusters_mv[i][jj];
                    const int kp_scene_idx = c.index_match;
                    const int kp_model_idx = c.index_query;
                    const PointT keypoint_model = accumulatedHypotheses_[id].correspondences_pointcloud->points[kp_model_idx];
                    const pcl::Normal keypoint_normal_model = accumulatedHypotheses_[id].normals_pointcloud->points[kp_model_idx];
                    const PointT keypoint_scene = pAccumulatedKeypoints_->points[kp_scene_idx];
                    const pcl::Normal keypoint_normal_scene = pAccumulatedKeypointNormals_->points[kp_scene_idx];
                    const int index_to_flann_models = accumulatedHypotheses_[id].indices_to_flann_models_[kp_model_idx];

                    // only add correspondences which correspondences to both, model and scene keypoint,
                    // are not already saved in the hypotheses coming from single view recognition only.
                    // As the keypoints from single view rec. are pushed in the front, we only have to check
                    // if the indices of the new correspondences are outside of these keypoint sizes.
                    // Also, we don't have to check if these new keypoints are redundant because this
                    // is already done in the function "extendFeatureMatchesRecursive(..)".

                    if(kp_model_idx >= it_hyp_sv->second.correspondences_pointcloud->points.size()
                            && kp_scene_idx >= grph_final_[vrtx_final].pKeypointsMultipipe_->points.size())
                    {
                        pcl::Correspondence c_new = c;  // to keep hypothesis' class union member distance
                        c_new.index_match = grph_final_[vrtx_final].pKeypointsMultipipe_->points.size();
                        c_new.index_query = it_hyp_sv->second.correspondences_pointcloud->points.size();
                        it_hyp_sv->second.correspondences_to_inputcloud->push_back(c_new);

                        it_hyp_sv->second.correspondences_pointcloud->points.push_back(keypoint_model);
                        it_hyp_sv->second.normals_pointcloud->points.push_back(keypoint_normal_model);
                        it_hyp_sv->second.indices_to_flann_models_.push_back(index_to_flann_models);
                        grph_final_[vrtx_final].pKeypointsMultipipe_->points.push_back(keypoint_scene);
                        grph_final_[vrtx_final].pKeypointNormalsMultipipe_->points.push_back(keypoint_normal_scene);
                    }
                }
            }
        }
    }

    if(visualize_output_)
        bgvis_.visualizeGraph(grph_final_, vis_);

    std::pair<vertex_iter, vertex_iter> vp;
    for ( vp = vertices ( grph_final_ ); vp.first != vp.second; ++vp.first )
    {   //--reset-hop-status
        grph_final_ [*vp.first].has_been_hopped_ = false;
    }

    grph_[vrtx] = grph_final_[vrtx_final]; // shallow copy is okay here

    //-------Clean-up-graph-------------------
    total_time = total_pcl_time.getTime();

    outputgraph ( grph_, "complete_graph.dot" );
    outputgraph ( grph_final_, "Final_with_Hypothesis_extension.dot" );
    pruneGraph(grph_, max_vertices_in_graph_);
    pruneGraph(grph_final_, max_vertices_in_graph_);

    outputgraph ( grph_final_, "final_after_deleting_old_vertex.dot" );
    outputgraph ( grph_, "grph_after_deleting_old_vertex.dot" );

    times_.push_back(total_time);
    times_.push_back(sv_hyp_construction_time);
    times_.push_back(sv_overhead_time);
    times_.push_back(mv_hyp_construct_time);
    times_.push_back(mv_hyp_ver_time);
    times_.push_back(mv_feat_ext_time);
    times_.push_back(mv_icp_time);
    times_.push_back(total_num_correspondences);
    times_.push_back(sv_num_correspondences);

    bgvis_.visualizeWorkflow(vrtx_final, grph_final_, pAccumulatedKeypoints_);
//    bgvis_.createImage(vrtx_final, grph_final_, "/home/thomas/Desktop/test.jpg");
    return true;
}
