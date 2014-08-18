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

inline void
createBigPointCloudRecursive ( Graph & grph_final, pcl::PointCloud<pcl::PointXYZRGB>::Ptr & big_cloud, Vertex start, Vertex coming_from,
                               Eigen::Matrix4f accum )
{
    if ( boost::degree ( start, grph_final ) == 1 )
    {
        //check if target is like coming_from
        boost::graph_traits<Graph>::out_edge_iterator ei, ei_end;
        for ( boost::tie ( ei, ei_end ) = boost::out_edges ( start, grph_final ); ei != ei_end; ++ei )
        {
            if ( target ( *ei, grph_final ) == coming_from )
                return;
        }
    }

    boost::graph_traits<Graph>::out_edge_iterator ei, ei_end;
    std::vector < boost::graph_traits<Graph>::out_edge_iterator > edges;
    for ( boost::tie ( ei, ei_end ) = boost::out_edges ( start, grph_final ); ei != ei_end; ++ei )
    {

        if ( target ( *ei, grph_final ) == coming_from )
        {
            continue;
        }

        edges.push_back ( ei );
    }

    for ( size_t i = 0; i < edges.size (); i++ )
    {
        Eigen::Matrix4f internal_accum;
        Edge e = *edges[i];
        Vertex src = boost::source ( e, grph_final );
        Vertex targ = boost::target ( e, grph_final );
        Eigen::Matrix4f transform;
        if ( grph_final[e].source_id.compare( grph_final[src].view_id_ ) == 0)
        {
            PCL_WARN ( "inverse" );
            transform = grph_final[e].transformation.inverse ();
        }
        else
        {
            PCL_WARN ( "normal" );
            transform = grph_final[e].transformation;
        }

        internal_accum = accum * transform;
        std::cout << internal_accum << std::endl;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr trans ( new pcl::PointCloud<pcl::PointXYZRGB> );
        pcl::transformPointCloud ( *grph_final[targ].pScenePCl, *trans, internal_accum );
        *big_cloud += *trans;
        grph_final[targ].absolute_pose = internal_accum;
        createBigPointCloudRecursive ( grph_final, big_cloud, targ, start, internal_accum );
    }
}

void  multiviewGraph::
createBigPointCloud ( Graph & grph_final, pcl::PointCloud<pcl::PointXYZRGB>::Ptr & big_cloud )
{
    std::pair<vertex_iter, vertex_iter> vp;
    vp = vertices ( grph_final );
    Eigen::Matrix4f accum;
    accum.setIdentity ();
    *big_cloud += *grph_final[*vp.first].pScenePCl;
    grph_final[*vp.first].absolute_pose = accum;
    createBigPointCloudRecursive ( grph_final, big_cloud, *vp.first, *vp.first, accum );
}


std::string multiviewGraph::models_dir() const
{
    return models_dir_;
}

void multiviewGraph::setModels_dir(const std::string &models_dir)
{
    models_dir_ = models_dir;
}

bool multiviewGraph::visualize_output() const
{
    return visualize_output_;
}

void multiviewGraph::setVisualize_output(bool visualize_output)
{
    visualize_output_ = visualize_output;
}

bool multiviewGraph::go_3d() const
{
    return go_3d_;
}

void multiviewGraph::setGo_3d(bool go_3d)
{
    go_3d_ = go_3d;
}

int multiviewGraph::icp_iter() const
{
    return icp_iter_;
}

void multiviewGraph::setIcp_iter(int icp_iter)
{
    icp_iter_ = icp_iter;
}

int multiviewGraph::opt_type() const
{
    return opt_type_;
}

void multiviewGraph::setOpt_type(int opt_type)
{
    opt_type_ = opt_type;
}

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

int multiviewGraph::mv_keypoints() const
{
    return mv_keypoints_;
}

void multiviewGraph::setMv_keypoints(int mv_keypoints)
{
    mv_keypoints_ = mv_keypoints;
}

void multiviewGraph::loadModels()
{
    //load models for visualization
    models_source_.reset ( new faat_pcl::rec_3d_framework::ModelOnlySource<pcl::PointXYZRGBNormal, PointT> );
    models_source_->setPath ( models_dir_ );
    models_source_->setLoadViews ( false );
    models_source_->setLoadIntoMemory ( false );
    std::string training_dir = "not_needed";
    models_source_->generate ( training_dir );
    models_source_->createVoxelGridAndDistanceTransform ( icp_resolution_ );
    ROS_INFO ( "Models loaded from %s", models_dir_.c_str() );

    if ( visualize_output_ ) //-------Visualize Scene Cloud--------------------
    {
        vis_.reset ( new pcl::visualization::PCLVisualizer ( "vis1" ) );
        vis_->setWindowName ( "Recognition from Multiple Views" );
    }
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
calcFeatures (Vertex &src, Graph &grph, bool use_table_plane )
{
    boost::shared_ptr < faat_pcl::rec_3d_framework::SIFTLocalEstimation<PointT, FeatureT> > estimator;
    estimator.reset (new faat_pcl::rec_3d_framework::SIFTLocalEstimation<PointT, FeatureT>(sift_));

    if(use_table_plane)
        estimator->setIndices (*(grph[src].pIndices_above_plane));

    bool ret = estimator->estimate (grph[src].pScenePCl_f, grph[src].pKeypoints, grph[src].pSignatures, grph[src].sift_keypoints_scales);

    estimator->getKeypointIndices(grph[src].keypoints_indices_);

    return ret;

    //----display-keypoints--------------------
    /*pcl::visualization::PCLVisualizer::Ptr vis_temp (new pcl::visualization::PCLVisualizer);
     pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> handler_rgb_verified (grph[*it_vrtx].pScenePCl);
     vis_temp->addPointCloud<pcl::PointXYZRGB> (grph[*it_vrtx].pScenePCl, handler_rgb_verified, "Hypothesis_1");
     pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> handler_rgb_verified2 (grph[*it_vrtx].pKeypoints);

     for (size_t keyId = 0; keyId < grph[*it_vrtx].pKeypoints->size (); keyId++)
     {
     std::stringstream sphere_name;
     sphere_name << "sphere_" << keyId;
     vis_temp->addSphere<pcl::PointXYZRGB> (grph[*it_vrtx].pKeypoints->at (keyId), 0.01, sphere_name.str ());
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
    int K = 1;
    flann::Matrix<int> indices = flann::Matrix<int> ( new int[K], 1, K );
    flann::Matrix<float> distances = flann::Matrix<float> ( new float[K], 1, K );
    PCL_INFO ( "Calculate transform via SIFT between view %s and %s for a keypoint size of %ld", grph[src].view_id_.c_str(), grph[trgt].view_id_.c_str(), grph[src].pKeypoints->size () );

    pcl::CorrespondencesPtr temp_correspondences ( new pcl::Correspondences );
    for ( size_t keypointId = 0; keypointId < grph[src].pKeypoints->size (); keypointId++ )
    {
        FeatureT searchFeature = grph[src].pSignatures->at ( keypointId );
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
        rej->setInputTarget (grph[trgt].pKeypoints);
        rej->setInputSource (grph[src].pKeypoints);
        rej->setInputCorrespondences (temp_correspondences);
        rej->getCorrespondences (*after_rej_correspondences);

        transformation = rej->getBestTransformation ();
        pcl::registration::TransformationEstimationSVD<PointT, PointT> t_est;
        t_est.estimateRigidTransformation (*grph[src].pKeypoints, *grph[trgt].pKeypoints, *after_rej_correspondences, transformation);

        std::cout << "size of corr before " << temp_correspondences->size () << "; after: " << after_rej_correspondences->size () << std::endl;

        bool b;
        Edge edge;
        tie (edge, b) = add_edge (trgt, src, grph);
        grph[edge].transformation = transformation;
        grph[edge].model_name = std::string ("scene_to_scene");
        grph[edge].source_id = grph[src].view_id_;
        grph[edge].target_id = grph[trgt].view_id_;
        edges.push_back(edge);
    }
    else
    {
        pcl::GeometricConsistencyGrouping<pcl::PointXYZRGB, pcl::PointXYZRGB> gcg_alg;
        gcg_alg.setGCThreshold (15);
        gcg_alg.setGCSize (0.01);
        gcg_alg.setInputCloud(grph[src].pKeypoints);
        gcg_alg.setSceneCloud(grph[trgt].pKeypoints);
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
            grph[edge].source_id = grph[src].view_id_;
            grph[edge].target_id = grph[trgt].view_id_;
            edges.push_back(edge);
        }
    }
}

void multiviewGraph::visualizeEdge (const Edge &edge, const Graph &grph)
{
    Vertex src = source ( edge, grph );
    Vertex trgt = target ( edge, grph );

    Eigen::Matrix4f transform;

    if ( grph[edge].source_id.compare( grph[src].view_id_ ) == 0)
    {
        transform = grph[edge].transformation;
    }
    else if (grph[edge].target_id.compare( grph[src].view_id_ ) == 0)
    {
        transform = grph[edge].transformation.inverse();
    }
    else
    {
        std::cout << "Something is messed up with the transformation! " << std::endl;
    }

    edge_vis->removeAllPointClouds();
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> handler_rgb_verified (grph[trgt].pScenePCl_f);
    edge_vis->addPointCloud<pcl::PointXYZRGB> (grph[trgt].pScenePCl_f, handler_rgb_verified, "Hypothesis_1");
    PointInTPtr transformed_PCl (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::transformPointCloud (*grph[src].pScenePCl_f, *transformed_PCl, transform);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> handler_rgb_verified2 (transformed_PCl);
    edge_vis->addPointCloud<pcl::PointXYZRGB> (transformed_PCl, handler_rgb_verified2, "Hypothesis_2");
    std::stringstream window_title;
    window_title << "transform of source view_id " << grph[src].view_id_ << " to target view_id " << grph[trgt].view_id_ << " with edge " << grph[edge].model_name;
    edge_vis->setWindowName(window_title.str());
    edge_vis->spin ();
}

void multiviewGraph::
estimateViewTransformationByRobotPose ( const Vertex &src, const Vertex &trgt, Graph &grph, Edge &edge )
{
    bool b;
    tie ( edge, b ) = add_edge ( trgt, src, grph );
    Eigen::Matrix4f tf2wco_src = grph[src].transform_to_world_co_system_;
    Eigen::Matrix4f tf2wco_trgt = grph[trgt].transform_to_world_co_system_;
    grph[edge].transformation = tf2wco_trgt.inverse() * tf2wco_src;
    grph[edge].model_name = std::string ( "robot_pose" );
    grph[edge].source_id = grph[src].view_id_;
    grph[edge].target_id = grph[trgt].view_id_;
}

//void multiviewGraph::
//selectLowestWeightEdgesFromParallelEdges ( const std::vector<Edge> &parallel_edges, const Graph &grph, std::vector<Edge> &single_edges )
//{
//    for ( size_t edgeVec_id = 0; edgeVec_id < parallel_edges.size (); edgeVec_id++ )
//    {
//        Vertex vrtx_src, vrtx_trgt;
//        vrtx_src = source ( parallel_edges[edgeVec_id], grph );
//        vrtx_trgt = target ( parallel_edges[edgeVec_id], grph );

//        bool found = false;
//        for ( size_t edges_lowestWeight_id = 0; edges_lowestWeight_id < single_edges.size (); edges_lowestWeight_id++ ) //select edge with lowest weight amongst parallel edges
//        {

//            //check if edge already exists in final graph between these two vertices
//            if ( ( ( ( boost::get ( vertex_index, grph, source ( single_edges[edges_lowestWeight_id], grph ) ) == boost::get ( vertex_index, grph, vrtx_src ) )
//                     && ( boost::get ( vertex_index, grph, target ( single_edges[edges_lowestWeight_id], grph ) ) == boost::get ( vertex_index, grph, vrtx_trgt ) ) )
//                   || ( ( boost::get ( vertex_index, grph, source ( single_edges[edges_lowestWeight_id], grph ) ) == boost::get ( vertex_index, grph, vrtx_trgt ) )
//                        && ( boost::get ( vertex_index, grph, target ( single_edges[edges_lowestWeight_id], grph ) ) == boost::get ( vertex_index, grph, vrtx_src ) ) ) ) )
//            {
//                found = true;
//                if ( grph[parallel_edges[edgeVec_id]].edge_weight < grph[single_edges[edges_lowestWeight_id]].edge_weight ) //check for lowest edge cost - if lower than currently lowest weight, then replace
//                {
//                    single_edges[edges_lowestWeight_id] = parallel_edges[edgeVec_id];
//                }
//                break;
//            }
//        }
//        if ( !found )
//            single_edges.push_back ( parallel_edges[edgeVec_id] );
//    }
//}

Vertex multiviewGraph::getFurthestVertex ( Graph &grph)
{
    std::pair<vertex_iter, vertex_iter> vp; //vp.first = running iterator..... vp.second = last iterator

    vp = vertices ( grph );
    Vertex furthest_vrtx = *vp.first;
    ++vp.first;

    for (; vp.first != vp.second; ++vp.first )
    {
        if(grph[*vp.first].cumulative_weight_to_new_vrtx_ > grph[furthest_vrtx].cumulative_weight_to_new_vrtx_)
        {
            furthest_vrtx = *vp.first;
        }
    }

    return furthest_vrtx;
}

std::vector<Hypothesis> multiviewGraph::
extendHypothesisRecursive ( Graph &grph, Edge calling_out_edge) //is directed edge (so the source of calling_edge is calling vertex)
{
    std::vector<Hypothesis> hyp_vec;

    typename graph_traits<Graph>::out_edge_iterator out_i, out_end;
    Vertex current_vertex = target ( calling_out_edge, grph );
    Vertex src = source ( calling_out_edge, grph );

    grph[current_vertex].has_been_hopped_ = true;
    grph[current_vertex].cumulative_weight_to_new_vrtx_ = grph[src].cumulative_weight_to_new_vrtx_ + grph[calling_out_edge].edge_weight;

    ROS_INFO("Current Vertex %s has a cumulative weight of %lf.", grph[current_vertex].view_id_.c_str(), grph[current_vertex].cumulative_weight_to_new_vrtx_);
    for ( tie ( out_i, out_end ) = out_edges ( current_vertex, grph ); out_i != out_end; ++out_i )
    {
        Vertex new_trgt = target ( *out_i, grph );

        if ( grph[new_trgt].has_been_hopped_ )
        {
            ROS_INFO("Vertex %s has already been hopped.", grph[new_trgt].view_id_.c_str());
            continue;
        }
        ROS_INFO("Hopping to vertex %s...", grph[new_trgt].view_id_.c_str());
        std::vector<Hypothesis> new_hypotheses = extendHypothesisRecursive ( grph, *out_i);
        for(std::vector<Hypothesis>::iterator it_new_hyp = new_hypotheses.begin(); it_new_hyp !=new_hypotheses.end(); ++it_new_hyp)
        {
            if ( grph[calling_out_edge].source_id.compare( grph[src].view_id_ ) == 0)
            {
                it_new_hyp->transform_ = grph[calling_out_edge].transformation.inverse () * it_new_hyp->transform_ ;
            }
            else if (grph[calling_out_edge].target_id.compare( grph[src].view_id_ ) == 0)
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

    for ( std::vector<Hypothesis>::const_iterator it_hyp = grph[current_vertex].hypothesis.begin (); it_hyp != grph[current_vertex].hypothesis.end (); ++it_hyp )
    {
        if(!it_hyp->verified_)
            continue;

        Eigen::Matrix4f tf;
        if ( grph[calling_out_edge].source_id.compare( grph[src].view_id_ ) == 0)
        {
            tf = grph[calling_out_edge].transformation.inverse () * it_hyp->transform_;
        }
        else if (grph[calling_out_edge].target_id.compare( grph[src].view_id_ ) == 0)
        {
            tf = grph[calling_out_edge].transformation * it_hyp->transform_;
        }
        else
        {
            ROS_WARN("Something is messed up with the transformation.");
        }
        Hypothesis ht_temp ( it_hyp->model_id_, tf, it_hyp->origin_, true );
        hyp_vec.push_back(ht_temp);
    }

    return hyp_vec;
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

                if ( grph[src].view_id_ != grph[*vp.first].view_id_ )
                    PCL_WARN("something's wrong");

                size_t hypothesis_length_before_extension = grph[src].hypothesis.size ();

                for ( std::vector<Hypothesis>::iterator it_hypB = grph[targ].hypothesis.begin (); it_hypB != grph[targ].hypothesis.end (); ++it_hypB )
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
                        if ( grph[e].source_id.compare( grph[*vp.first].view_id_ ) == 0 )
                        {
                            tf = grph[e].transformation.inverse () * it_hypB->transform_;
                        }
                        else
                        {
                            tf = grph[e].transformation * it_hypB->transform_;
                        }

                        Hypothesis ht_temp ( it_hypB->model_id_, tf, it_hypB->origin_, true );
                        grph[*vp.first].hypothesis.push_back ( ht_temp );
                        something_has_been_updated = true;
                    }
                }
            }
        }
    }
}

//void multiviewGraph::
//calcMST ( const std::vector<Edge> &edges, const Graph &grph, std::vector<Edge> &edges_final )
//{
//    GraphMST grphMST;
//    std::vector<VertexMST> verticesMST_v;
//    std::vector<Edge> edges_lowestWeight;

//    for ( std::pair<vertex_iter, vertex_iter> vp = vertices ( grph ); vp.first != vp.second; ++vp.first )
//    {
//        VertexMST vrtxMST = boost::add_vertex ( grphMST );
//        verticesMST_v.push_back ( vrtxMST );
//    }

//    selectLowestWeightEdgesFromParallelEdges ( edges, grph, edges_lowestWeight );

//    //---create-input-for-Minimum-Spanning-Tree-calculation-------------------------------
//    for ( size_t edgeVec_id = 0; edgeVec_id < edges_lowestWeight.size (); edgeVec_id++ )
//    {
//        Vertex vrtx_src, vrtx_trgt;
//        vrtx_src = source ( edges_lowestWeight[edgeVec_id], grph );
//        vrtx_trgt = target ( edges_lowestWeight[edgeVec_id], grph );
//        add_edge ( verticesMST_v[get ( vertex_index, grph, vrtx_src )], verticesMST_v[get ( vertex_index, grph, vrtx_trgt )],
//                grph[edges_lowestWeight[edgeVec_id]].edge_weight, grphMST );
//    }

//#if defined(BOOST_MSVC) && BOOST_MSVC <= 1300
//    std::cout << "Boost Version not supported (you are using BOOST_MSVC Version: " << BOOST_MSVC << ", BOOST_MSVC > 1300 needed)" << std::endl;
//#else
//    std::vector < graph_traits<GraphMST>::vertex_descriptor > p ( num_vertices ( grphMST ) );
//    prim_minimum_spanning_tree ( grphMST, &p[0] );

//    dynamic_properties dp;
//    dp.property ( "node_id", get ( vertex_index, grphMST ) );
//    dp.property ( "weight", get ( edge_weight, grphMST ) );
//    std::cout << "Result Prims Algorithm: \n======================" << std::endl;
//    write_graphviz_dp ( std::cout, grphMST, dp, "node_id" );
//    std::cout << " There are " << boost::num_edges ( grphMST ) << " edges in the graph grph." << std::endl;

//#endif

//    for ( std::size_t i = 0; i != p.size (); ++i )
//    {
//        if ( p[i] != i )
//            std::cout << "parent[" << i << "] = " << p[i] << std::endl;
//        else
//            std::cout << "parent[" << i << "] = no parent" << std::endl;
//    }

//    for ( size_t edgeVec_id = 0; edgeVec_id < edges_lowestWeight.size (); edgeVec_id++ )
//    {
//        Vertex vrtx_src, vrtx_trgt;
//        vrtx_src = source ( edges_lowestWeight[edgeVec_id], grph );
//        vrtx_trgt = target ( edges_lowestWeight[edgeVec_id], grph );
//        if ( p[boost::get ( vertex_index, grph, vrtx_src )] == boost::get ( vertex_index, grph, vrtx_trgt ) || p[boost::get ( vertex_index, grph, vrtx_trgt )]
//             == boost::get ( vertex_index, grph, vrtx_src ) ) //check if edge represents an edge of Prim's Minimum Spanning Tree
//        {
//            edges_final.push_back ( edges_lowestWeight[edgeVec_id] );
//        }
//    }
//}

//void multiviewGraph::
//createEdgesFromHypothesisMatch (Graph &grph, std::vector<Edge> &edges )
//{
//    vertex_iter vertexItA, vertexEndA;
//    for (boost::tie(vertexItA, vertexEndA) = vertices(grph_); vertexItA != vertexEndA; ++vertexItA)
//    {
//        for ( size_t hypVec_id = 0; hypVec_id < grph[*vertexItA].hypothesis.size (); hypVec_id++ )
//        {
//            vertex_iter vertexItB, vertexEndB;
//            for (boost::tie(vertexItB, vertexEndB) = vertices(grph_); vertexItB != vertexItA; ++vertexItB)
//            {
//                //was for ( std::vector<Vertex>::const_iterator it_vrtxB = vertices_v.begin (); it_vrtxB != it_vrtxA; ++it_vrtxB )
//                for ( std::vector<Hypothesis>::iterator it_hypB = grph[*vertexItB].hypothesis.begin (); it_hypB != grph[*vertexItB].hypothesis.end (); ++it_hypB )
//                {
//                    if ( it_hypB->model_id_.compare ( grph[*vertexItA].hypothesis[hypVec_id].model_id_ ) == 0 ) //model exists in other file -> create connection
//                    {
//                        Eigen::Matrix4f tf_temp = it_hypB->transform_ * grph[*vertexItA].hypothesis[hypVec_id].transform_.inverse (); //might be the other way around

//                        //link views by an edge (for other graph)
//                        Edge e_cpy;
//                        bool b;
//                        tie ( e_cpy, b ) = add_edge ( *vertexItA, *vertexItB, grph );
//                        grph[e_cpy].transformation = tf_temp;
//                        grph[e_cpy].model_name = grph[*vertexItA].hypothesis[hypVec_id].model_id_;
//                        grph[e_cpy].source_id = grph[*vertexItA].view_id_;
//                        grph[e_cpy].target_id = grph[*vertexItB].view_id_;
//                        grph[e_cpy].edge_weight = std::numeric_limits<double>::max ();
//                        edges.push_back ( e_cpy );

//                        std::cout << "Creating Edge from view " << grph[*vertexItA].view_id_ << " to " << grph[*vertexItB].view_id_
//                                  << std::endl;
//                    }
//                }
//            }
//        }
//    }
//}

void multiviewGraph::
createEdgesFromHypothesisMatchOnline ( const Vertex new_vertex, Graph &grph, std::vector<Edge> &edges )
{
    vertex_iter vertexItA, vertexEndA;
    for (boost::tie(vertexItA, vertexEndA) = vertices(grph_); vertexItA != vertexEndA; ++vertexItA)
    {
        if ( grph[*vertexItA].view_id_.compare( grph[new_vertex].view_id_ ) == 0 )
        {
            continue;
        }

        ROS_INFO("Checking vertex %s, which has %ld hypotheses.", grph[*vertexItA].view_id_.c_str(), grph[*vertexItA].hypothesis.size());
        for ( std::vector<Hypothesis>::iterator it_hypA = grph[*vertexItA].hypothesis.begin (); it_hypA != grph[*vertexItA].hypothesis.end (); ++it_hypA )
        {
            for ( std::vector<Hypothesis>::iterator it_hypB = grph[new_vertex].hypothesis.begin (); it_hypB != grph[new_vertex].hypothesis.end (); ++it_hypB )
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
                    grph[e_cpy].source_id = grph[*vertexItA].view_id_;
                    grph[e_cpy].target_id = grph[new_vertex].view_id_;
                    grph[e_cpy].edge_weight = std::numeric_limits<double>::max ();
                    edges.push_back ( e_cpy );

                    std::cout << "Creating edge from view " << grph[*vertexItA].view_id_ << " to view " << grph[new_vertex].view_id_
                              << " for model match " << grph[e_cpy].model_name
                              << std::endl;
                }
            }
        }
    }
}

void multiviewGraph::
calcEdgeWeight ( Graph &grph, int max_distance, float z_dist, float max_overlap)
{
    //----calculate-edge-weight---------------------------------------------------------

    std::pair<edge_iter, edge_iter> ep = edges(grph);
    for (; ep.first!=ep.second; ++ep.first) //std::vector<Edge>::iterator edge_it = edges.begin(); edge_it!=edges.end(); ++edge_it)
    {
        if(grph[*ep.first].edge_weight_has_been_calculated_)
            continue;

        double edge_weight;
        Vertex vrtx_src, vrtx_trgt;
        vrtx_src = source ( *ep.first, grph );
        vrtx_trgt = target ( *ep.first, grph );

        Eigen::Matrix4f transform;
        if ( grph[*ep.first].source_id.compare( grph[vrtx_src].view_id_ ) == 0)
        {
            transform = grph[*ep.first].transformation;
        }
        else
        {
            transform = grph[*ep.first].transformation.inverse ();
        }

        float w_after_icp_ = std::numeric_limits<float>::max ();
        float best_overlap_ = max_overlap;

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

        //        pcl::transformPointCloudWithNormals ( * ( grph[vrtx_src].pSceneXYZRGBNormal ), *pTargetNormalPCl, icp_trans );
        //        pcl::copyPointCloud ( * ( grph[vrtx_trgt].pSceneXYZRGBNormal ), *pSourceNormalPCl );

        if ( grph[*ep.first].source_id.compare( grph[vrtx_src].view_id_ ) == 0)
        {
            PCL_WARN ( "Normal...\n" );
            //icp trans is aligning source to target
            //transform is aligning source to target
            //grph[edges[edge_id]].transformation = icp_trans * grph[edges[edge_id]].transformation;
            grph[*ep.first].transformation = icp_trans;
        }
        else
        {
            //transform is aligning target to source
            //icp trans is aligning source to target
            PCL_WARN ( "Inverse...\n" );
            //grph[edges[edge_id]].transformation = icp_trans.inverse() * grph[edges[edge_id]].transformation;
            grph[*ep.first].transformation = icp_trans.inverse ();
        }

        grph[*ep.first].edge_weight = w_after_icp_;
        grph[*ep.first].edge_weight_has_been_calculated_ = true;

        std::cout << "WEIGHT IS: " << grph[*ep.first].edge_weight << " coming from edge connecting " << grph[*ep.first].source_id
                  << " and " << grph[*ep.first].target_id << " by object_id: " << grph[*ep.first].model_name
                  << std::endl;
    }
}

Eigen::Matrix4f GeometryMsgToMatrix4f(geometry_msgs::Transform & tt)
{
    Eigen::Matrix4f trans = Eigen::Matrix4f::Identity();
    trans(0,3) = tt.translation.x;
    trans(1,3) = tt.translation.y;
    trans(2,3) = tt.translation.z;

    Eigen::Quaternionf q(tt.rotation.w,tt.rotation.x,tt.rotation.y,tt.rotation.z);
    trans.block<3,3>(0,0) = q.toRotationMatrix();
    return trans;
}

bool multiviewGraph::recognize
                (pcl::PointCloud<pcl::PointXYZRGB>::Ptr pInputCloud,
                 std::vector<Eigen::Matrix4f> &hyp_transforms_local,
                 std::vector<std::string> &hyp_model_ids,
                 const std::string view_name,
                 const Eigen::Matrix4f global_transform,
                 const size_t timestamp)
{
//    ;
//}

//bool multiviewGraph::recognize (recognition_srv_definitions::multiview_recognize::Request & req, recognition_srv_definitions::multiview_recognize::Response & response) // pcl::PointCloud<pcl::PointXYZRGB> &cloud, const std::string scene_name )
//{
    assert(pInputCloud->width == 640 && pInputCloud->height == 480);
    Vertex vrtx = boost::add_vertex ( grph_ );

//    Eigen::Matrix4f trans;
//    for (size_t row=0; row <4; row++)
//    {
//        for(size_t col=0; col<4; col++)
//        {
//            trans(row, col) = req.transform[4*row + col];
//        }
//    }

    grph_[vrtx].pScenePCl = pInputCloud;
    grph_[vrtx].transform_to_world_co_system_ = global_transform;
    grph_[vrtx].view_id_ = view_name;
    grph_[vrtx].timestamp_nsec = timestamp;


    if(chop_at_z_ > 0)
    {
        pcl::PassThrough<PointT> pass_;
        pass_.setFilterLimits (0.f, static_cast<float>(chop_at_z_));
        pass_.setFilterFieldName ("z");
        pass_.setInputCloud (grph_[vrtx].pScenePCl);
        pass_.setKeepOrganized (true);
        pass_.filter (*(grph_[vrtx].pScenePCl_f));
    }

    std::vector<Edge> new_edges;

    //--------------create-edges-between-views-by-Robot-Pose-----------------------------
    if(use_robot_pose_ && num_vertices(grph_)>1)
    {
        vertex_iter vertexIt, vertexEnd;
        for (boost::tie(vertexIt, vertexEnd) = vertices(grph_); vertexIt != vertexEnd; ++vertexIt)
        {
            Edge edge;
            if( grph_[*vertexIt].view_id_.compare ( grph_[vrtx].view_id_ ) != 0 )
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
        calcFeatures ( vrtx, grph_ );
        std::cout << "keypoints: " << grph_[vrtx].pKeypoints->points.size() << std::endl;

        if (num_vertices(grph_)>1)
        {
            flann::Matrix<float> flann_data;
            flann::Index<DistT> *flann_index;
            multiview::convertToFLANN<pcl::Histogram<128> > ( grph_[vrtx].pSignatures, flann_data );
            flann_index = new flann::Index<DistT> ( flann_data, flann::KDTreeIndexParams ( 4 ) );
            flann_index->buildIndex ();

            //#pragma omp parallel for
            vertex_iter vertexIt, vertexEnd;
            for (boost::tie(vertexIt, vertexEnd) = vertices(grph_); vertexIt != vertexEnd; ++vertexIt)
            {
                Eigen::Matrix4f transformation;
                if( grph_[*vertexIt].view_id_.compare ( grph_[vrtx].view_id_ ) != 0 )
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
    std::vector<Eigen::Matrix4f> trans_v;
    std::vector<std::string> model_ids_v;
    pSingleview_recognizer_->recognize(grph_[vrtx].pScenePCl_f, trans_v, model_ids_v);
    if(model_ids_v.size())
    {
        for(size_t model_id=0; model_id < model_ids_v.size(); model_id++)
        {
            std::cout << "I detected object " <<
                        model_ids_v[model_id]<< " in the scene." << std::endl;

            std::stringstream model_name;
            model_name << models_dir_ << model_ids_v[model_id];
            Hypothesis hypothesis ( model_name.str(), trans_v[model_id], grph_[vrtx].view_id_, false );
            grph_[vrtx].hypothesis.push_back ( hypothesis );
        }
    }
    else
    {
        std::cout << "I didn't detect any objects in the current scene." << std::endl;
    }
    //    if ( client_.call ( srv ) )
    //    {
    //        if ( srv.response.ids.size() == 0 )
    //        {
    //            ROS_INFO ( "I didn't detect any object in the current scene." );
    //        }
    //        else
    //        {
    //            for ( size_t i=0; i < srv.response.ids.size(); i++ )
    //            {
    //                std_msgs::String object_id = ( std_msgs::String ) srv.response.ids[i];
    //                ROS_INFO ( "I detected object %s in the scene.", object_id.data.c_str() );

    //                Eigen::Matrix4f tt = GeometryMsgToMatrix4f(srv.response.transforms[i]);

    //                std::stringstream model_name;
    //                model_name << models_dir_ << srv.response.ids[i].data;
    //                Hypothesis hypothesis ( model_name.str(), tt, grph_[vrtx].view_id_, false );
    //                grph_[vrtx].hypothesis.push_back ( hypothesis );
    //            }
    //        }
    //    }
    //    else
    //    {
    //        ROS_ERROR ( "Failed to call service" );
    //    }
    //----------END-call-single-view-recognizer------------------------------------------


    //-----Normal estimation actually redundant because is also calculated in the single view recognition service-----------
    pcl::NormalEstimationOMP<PointT, pcl::Normal> ne;
    ne.setRadiusSearch ( 0.02f );
    ne.setInputCloud ( grph_[vrtx].pScenePCl );
    ne.compute ( * ( grph_[vrtx].pSceneNormals ) );

    createEdgesFromHypothesisMatchOnline ( vrtx, grph_, new_edges );
    calcEdgeWeight (grph_);
    outputgraph ( grph_, "complete_graph.dot" );

    //---copy-vertices-to-graph_final----------------------------
    Vertex vrtx_final = boost::add_vertex ( grph_final_ );
    copyVertexIntoOtherGraph(vrtx, grph_, vrtx_final, grph_final_);


    //calcMST (edges_, grph, edges_final);      //No Minimum Spanning Tree calculation at the moment


    if ( num_vertices(grph_) > 1 )
    {
        //------find best edge from the freshly inserted view and add it to the final graph-----------------
        Edge best_edge;
        best_edge = new_edges[0];

        for ( size_t new_edge_id = 0; new_edge_id < new_edges.size(); new_edge_id ++ )
        {
            visualizeEdge(new_edges[new_edge_id], grph_);
            edges_.push_back ( new_edges[new_edge_id] );

            if ( grph_[new_edges[new_edge_id]].edge_weight < grph_[best_edge].edge_weight )
            {
                best_edge = new_edges[new_edge_id];
            }
        }

        Vertex vrtx_src, vrtx_trgt;
        vrtx_src = source ( best_edge, grph_ );
        vrtx_trgt = target ( best_edge, grph_ );

        Edge e_cpy; bool b;
        tie ( e_cpy, b ) = add_edge ( vrtx_src, vrtx_trgt, grph_final_ );
        copyEdgeIntoOtherGraph(best_edge, grph_, e_cpy, grph_final_);
        best_edges_.push_back ( best_edge );
        outputgraph ( grph_final_, "prune_graph.dot" );


        //---------Extend-hypotheses-from-other-view(s)------------------------------------------
        typename graph_traits<Graph>::out_edge_iterator out_i, out_end;
        tie ( out_i, out_end ) = out_edges ( vrtx_final, grph_final_ );    //There should only be one edge

        if ( out_i != out_end)
            std::cout << "Best edge: " << grph_final_[*out_i].edge_weight << " coming from edge " << grph_final_[*out_i].model_name << std::endl;
        else
        {
            std::cerr << "Why is there more than 1 edge for the last vertex?" << std::endl;
            for (; out_i != out_end; ++out_i )
            {
                std::cout << "Edge src: " << grph_final_[*out_i].source_id << "; target: " << grph_final_[*out_i].target_id << "; model_name: " << grph_final_[*out_i].model_name
                          << "; edge_weight: " << grph_final_[*out_i].edge_weight << std::endl;
            }
        }

        grph_final_[vrtx_final].has_been_hopped_ = true;
        std::vector<Hypothesis> all_hypotheses = extendHypothesisRecursive ( grph_final_, *out_i );

        for(std::vector<Hypothesis>::const_iterator it_all_hyp = all_hypotheses.begin(); it_all_hyp != all_hypotheses.end(); ++it_all_hyp)
        {
            grph_[vrtx].hypothesis.push_back(*it_all_hyp);
            grph_final_[vrtx_final].hypothesis.push_back(*it_all_hyp);
        }
        ROS_INFO ("There are %ld hypotheses in the current view after extension, whereby %ld have been extended.",
                  grph_final_[vrtx_final].hypothesis.size(), all_hypotheses.size());

        grph_[vrtx].cumulative_weight_to_new_vrtx_ = grph_final_[vrtx_final].cumulative_weight_to_new_vrtx_;

        std::pair<vertex_iter, vertex_iter> vp;
        for ( vp = vertices ( grph_final_ ); vp.first != vp.second; ++vp.first )
        {   //--reset-hop-status
            grph_final_ [*vp.first].has_been_hopped_ = false;
        }


        //-------------Refine hypotheses that where extended-------------------------
        if ( icp_iter_ > 0 )
        {
            std::pair<vertex_iter, vertex_iter> vp;
            bool current_iteration_done = false;
            for ( vp = vertices ( grph_final_ ); (vp.first != vp.second) && (!current_iteration_done); ++vp.first )
            {
                Vertex vrtx_tmp;

                if ( ! do_reverse_hyp_extension)	// It only applies ICP on the latest scene point cloud
                {
                    vrtx_tmp = vrtx_final;
                    current_iteration_done = true;
                }
                else
                {
                    vrtx_tmp = *vp.first;
                }
                std::cout << "Hypotheses in this frame after extension:" << grph_final_[vrtx_tmp].hypothesis.size () << std::endl;

#pragma omp parallel for num_threads(8)
                for ( size_t kk = 0; kk < grph_final_[vrtx_tmp].hypothesis.size (); kk++ )
                {
                    if ( !grph_final_[vrtx_tmp].hypothesis[kk].extended_ )
                        continue;

                    std::vector < std::string > strs_2;
                    boost::split ( strs_2, grph_final_[vrtx_tmp].hypothesis[kk].model_id_, boost::is_any_of ( "/\\" ) );
                    ModelTPtr model;
                    bool found = models_source_->getModelById ( strs_2[strs_2.size () - 1], model );

                    if ( found )
                    {
                        boost::shared_ptr < distance_field::PropagationDistanceField<pcl::PointXYZRGB> > dt;
                        model->getVGDT ( dt );

                        faat_pcl::rec_3d_framework::VoxelBasedCorrespondenceEstimation<pcl::PointXYZRGB, pcl::PointXYZRGB>::Ptr
                                est (
                                    new faat_pcl::rec_3d_framework::VoxelBasedCorrespondenceEstimation<
                                    pcl::PointXYZRGB,
                                    pcl::PointXYZRGB> () );

                        pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZRGB>::Ptr
                                rej (
                                    new pcl::registration::CorrespondenceRejectorSampleConsensus<
                                    pcl::PointXYZRGB> () );

                        Eigen::Matrix4f scene_to_model_trans = grph_final_[vrtx_tmp].hypothesis[kk].transform_.inverse ();

                        pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud;
                        dt->getInputCloud ( cloud );

                        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_voxelized_icp_transformed ( new pcl::PointCloud<pcl::PointXYZRGB> () );
                        pcl::transformPointCloud ( *grph_final_[vrtx_tmp].pScenePCl_f, *cloud_voxelized_icp_transformed, scene_to_model_trans );

                        est->setVoxelRepresentationTarget ( dt );
                        est->setInputSource ( cloud_voxelized_icp_transformed );
                        est->setInputTarget ( cloud );
                        est->setMaxCorrespondenceDistance ( icp_max_correspondence_distance_ );
                        est->setMaxColorDistance ( -1, -1 );

                        rej->setInputTarget ( cloud );
                        rej->setMaximumIterations ( 1000 );
                        rej->setInlierThreshold ( 0.005f );
                        rej->setInputSource ( cloud_voxelized_icp_transformed );

                        pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> reg;
                        reg.setCorrespondenceEstimation ( est );
                        reg.addCorrespondenceRejector ( rej );
                        reg.setInputTarget ( cloud ); //model
                        reg.setInputSource ( cloud_voxelized_icp_transformed ); //scene
                        reg.setMaximumIterations ( icp_iter_ );
                        reg.setEuclideanFitnessEpsilon ( 1e-12 );
                        reg.setTransformationEpsilon ( 0.0001f * 0.0001f );

                        pcl::registration::DefaultConvergenceCriteria<float>::Ptr convergence_criteria;
                        convergence_criteria = reg.getConvergeCriteria ();
                        convergence_criteria->setAbsoluteMSE ( 1e-12 );
                        convergence_criteria->setMaximumIterationsSimilarTransforms ( 15 );
                        convergence_criteria->setFailureAfterMaximumIterations ( false );

                        PointInTPtr output ( new pcl::PointCloud<pcl::PointXYZRGB> () );
                        reg.align ( *output );
                        Eigen::Matrix4f trans, icp_trans;
                        trans = reg.getFinalTransformation () * scene_to_model_trans;
                        icp_trans = trans.inverse ();
                        grph_final_[vrtx_tmp].hypothesis[kk].transform_ = icp_trans;
                    }
                }
            }
        }


        if (go_3d_)
        {
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr big_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
            createBigPointCloud (grph_final_, big_cloud);

            std::pair<vertex_iter, vertex_iter> vp;
            std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> object_clouds; //for MV ICP
            std::vector<pcl::PointCloud<pcl::Normal>::Ptr> normals_clouds; //for MV ICP
            std::vector< std::vector<float> > mv_weights; //for MV ICP

            std::vector< std::vector<float> > views_noise_weights;
            std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> original_clouds;
            std::vector<pcl::PointCloud<pcl::Normal>::Ptr> normal_clouds;

            bool use_normals = true;
            int idx=0;
            std::vector<pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr> occlusion_clouds;

            for (vp = vertices (grph_final_); vp.first != vp.second; ++vp.first, ++idx)
            {

                pcl::PointCloud<pcl::PointXYZRGB>::Ptr trans_cloud (new pcl::PointCloud<pcl::PointXYZRGB>(*grph_final_[*vp.first].pScenePCl));
                pcl::PointCloud<pcl::Normal>::Ptr normal_cloud (new pcl::PointCloud<pcl::Normal>(*grph_final_[*vp.first].pSceneNormals));

                normal_clouds.push_back(grph_final_[*vp.first].pSceneNormals);

                std::vector<bool> kept_map;
                kept_map.resize(trans_cloud->points.size(), false);

                std::vector<int> kept_indices;
                {
                    bool depth_edges = true;

                    faat_pcl::utils::noise_models::NguyenNoiseModel<pcl::PointXYZRGB> nm;
                    nm.setInputCloud(trans_cloud);
                    nm.setInputNormals(normal_cloud);
                    nm.setLateralSigma(lateral_sigma_);
                    nm.setMaxAngle(max_angle_);
                    nm.setUseDepthEdges(depth_edges);
                    nm.compute();

                    std::vector<float> ws;
                    nm.getWeights(ws);
                    views_noise_weights.push_back(ws);

                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered;
                    nm.getFilteredCloudRemovingPoints(filtered, 0.8f, kept_indices);

                    for(size_t i=0; i < kept_indices.size(); i++)
                    {
                        kept_map[kept_indices[i]] = true;
                        float dist = trans_cloud->points[kept_indices[i]].getVector3fMap().norm();
                        if(dist > max_keypoint_dist_mv_)
                        {
                            kept_map[kept_indices[i]] = false;
                        }
                    }
                }

                int kept=0;
                for(size_t i=0; i < kept_map.size(); i++)
                {
                    if(kept_map[i])
                        kept++;
                }

                std::cout << "kept:" << kept << " " << max_keypoint_dist_mv_ << std::endl;

                if(use_table_plane_)
                {
                    Eigen::Vector4f table_plane;
                    faat_pcl::utils::computeTablePlane<pcl::PointXYZRGB>(grph_final_[*vp.first].pScenePCl_f, table_plane);
                    for (size_t kk = 0; kk < trans_cloud->points.size (); kk++)
                    {

                        Eigen::Vector3f xyz_p = trans_cloud->points[kk].getVector3fMap ();

                        if (!pcl_isfinite ( xyz_p[0] ) || !pcl_isfinite ( xyz_p[1] ) || !pcl_isfinite ( xyz_p[2] ))
                            continue;

                        float val = xyz_p[0] * table_plane[0] + xyz_p[1] * table_plane[1] + xyz_p[2] * table_plane[2] + table_plane[3];

                        if (val <= -0.01 || !kept_map[kk])
                        {
                            trans_cloud->points[kk].x = std::numeric_limits<float>::quiet_NaN ();
                            trans_cloud->points[kk].y = std::numeric_limits<float>::quiet_NaN ();
                            trans_cloud->points[kk].z = std::numeric_limits<float>::quiet_NaN ();
                        }
                    }
                }
                else
                {
                    for (size_t kk = 0; kk < trans_cloud->points.size (); kk++)
                    {
                        if(!kept_map[kk])
                        {
                            trans_cloud->points[kk].x = std::numeric_limits<float>::quiet_NaN ();
                            trans_cloud->points[kk].y = std::numeric_limits<float>::quiet_NaN ();
                            trans_cloud->points[kk].z = std::numeric_limits<float>::quiet_NaN ();
                        }
                    }
                }

                pcl::PointCloud<pcl::Normal>::Ptr normal_cloud_trans (new pcl::PointCloud<pcl::Normal>());
                faat_pcl::utils::miscellaneous::transformNormals(normal_cloud, normal_cloud_trans, grph_final_[*vp.first].absolute_pose);

                if(mv_keypoints_ == 0)
                    //using SIFT keypoints
                {
                    //compute indices to original cloud (to address normals) that are not farther away that 1.3
                    std::vector<int> sift_indices = grph_final_[*vp.first].keypoints_indices_.indices;
                    std::vector<int> original_indices;
                    std::vector<float> keypoint_scales = grph_final_[*vp.first].sift_keypoints_scales;

                    for(size_t kk=0; kk < sift_indices.size(); kk++)
                    {
                        float dist = trans_cloud->points[sift_indices[kk]].getVector3fMap().norm();
                        if(dist > max_keypoint_dist_mv_)
                            continue;

                        if(!pcl_isfinite(trans_cloud->points[sift_indices[kk]].z))
                            continue;

                        /*std::cout << "scale:" << keypoint_scales[kk] << std::endl;
                        if(keypoint_scales[kk] > 1.5f)
                            continue;*/

                        original_indices.push_back(sift_indices[kk]);
                    }

                    std::cout << "SIFT keypoints:" << sift_indices.size() << " " << trans_cloud->points.size() << " " << original_indices.size() << std::endl;

                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr trans_cloud2 (new pcl::PointCloud<pcl::PointXYZRGB>());
                    pcl::copyPointCloud(*trans_cloud, original_indices, *trans_cloud2);

                    std::vector<float> view_weights;
                    mv_weights.push_back(view_weights);
                    float start = 1.f;
                    float end=3.f;

                    for(size_t kk=0; kk < trans_cloud2->points.size(); kk++)
                    {
                        float capped_dist = std::min(std::max(start, trans_cloud2->points[kk].z), end); //[start,end]
                        float w =  1.f - (capped_dist - start) / (end - start);
                        mv_weights[idx].push_back(w);
                    }

                    pcl::transformPointCloud(*trans_cloud2, *trans_cloud, grph_final_[*vp.first].absolute_pose);
                    pcl::copyPointCloud(*normal_cloud_trans, original_indices, *normal_cloud);
                }
                else if(mv_keypoints_ == 1)
                {
                    //using RGB edges
                    std::vector<int> edge_indices;
                    registration_utils::getRGBEdges<pcl::PointXYZRGB>(grph_final_[*vp.first].pScenePCl, edge_indices, 175, 225, max_keypoint_dist_mv_);
                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr trans_cloud2 (new pcl::PointCloud<pcl::PointXYZRGB>);
                    //pcl::copyPointCloud(*grph_final[*vp.first].pScenePCl, edge_indices, *trans_cloud2);

                    std::vector<int> final_indices;
                    for(size_t kk=0; kk < edge_indices.size(); kk++)
                    {
                        float dist = trans_cloud->points[edge_indices[kk]].getVector3fMap().norm();
                        if(dist > max_keypoint_dist_mv_)
                            //if(trans_cloud->points[edge_indices[kk]].z > max_keypoint_dist_mv_)
                            continue;

                        if(!pcl_isfinite(trans_cloud->points[edge_indices[kk]].z))
                            continue;

                        final_indices.push_back(edge_indices[kk]);
                    }

                    pcl::copyPointCloud(*trans_cloud, final_indices, *trans_cloud2);
                    pcl::transformPointCloud(*trans_cloud2, *trans_cloud, grph_final_[*vp.first].absolute_pose);
                    pcl::copyPointCloud(*normal_cloud_trans, final_indices, *normal_cloud);
                }
                else
                {
                    use_normals = false;
                    pcl::PassThrough<pcl::PointXYZRGB> pass;
                    pass.setInputCloud(trans_cloud);
                    pass.setFilterLimits(0, max_keypoint_dist_mv_);
                    pass.setFilterFieldName("z");
                    pass.setKeepOrganized(false);

                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr trans_cloud2 (new pcl::PointCloud<pcl::PointXYZRGB>);
                    pass.filter(*trans_cloud2);
                    pcl::transformPointCloud(*trans_cloud2, *trans_cloud, grph_final_[*vp.first].absolute_pose);
                }

                pcl::PointCloud<pcl::PointXYZRGB>::Ptr trans_cloud2 (new pcl::PointCloud<pcl::PointXYZRGB>(*trans_cloud));

                original_clouds.push_back(grph_final_[*vp.first].pScenePCl);
                object_clouds.push_back(trans_cloud2);
                normals_clouds.push_back(normal_cloud);
            }

            float total_keypoints = 0;
            for (size_t i = 0; i < object_clouds.size (); i++)
                total_keypoints += object_clouds[i]->points.size();

            std::cout << "Total number of keypoints:" << total_keypoints << std::endl;

            float dt_size = 0.002f;

            std::vector < Eigen::Matrix4f > transformations;
            transformations.resize(object_clouds.size(), Eigen::Matrix4f::Identity());

            if(mv_icp_)
            {
                //refine registered scene clouds simulatenously and adapt transforms
                std::vector < std::vector<bool> > A;
                A.resize (object_clouds.size ());
                for (size_t i = 0; i < object_clouds.size (); i++)
                    A[i].resize (object_clouds.size (), true);

                faat_pcl::registration_utils::computeOverlapMatrix<pcl::PointXYZRGB>(object_clouds, A, 0.02, false, min_overlap_mv_);

                for (size_t i = 0; i < object_clouds.size (); i++)
                {
                    for (size_t j = 0; j < object_clouds.size (); j++)
                        std::cout << (int)A[i][j] << " ";
                    std::cout << std::endl;
                }

                faat_pcl::registration::MVNonLinearICP<PointT> icp_nl (dt_size);
                icp_nl.setInlierThreshold (inlier_threshold_);
                icp_nl.setMaxCorrespondenceDistance (max_corresp_dist_);
                icp_nl.setClouds (object_clouds);

                if(use_normals)
                {
                    icp_nl.setInputNormals(normals_clouds);
                    icp_nl.setMinDot(0.9f);
                }

                icp_nl.setVisIntermediate (false);
                icp_nl.setSparseSolver (true);
                icp_nl.setMaxIterations(mv_iterations_);
                icp_nl.setAdjacencyMatrix (A);

                if(mv_weights.size() == object_clouds.size())
                    icp_nl.setPointsWeight(mv_weights);

                icp_nl.compute ();

                icp_nl.getTransformation (transformations);

                int kk=0;
                for (vp = vertices (grph_final_); vp.first != vp.second; ++vp.first, kk++)
                {
                    grph_final_[*vp.first].absolute_pose = transformations[kk] * grph_final_[*vp.first].absolute_pose;
                }
            }

            /*if(visualize_output)
            {

                pcl::PointCloud<pcl::PointXYZRGB>::Ptr big_cloud_after_mv_unfiltered (new pcl::PointCloud<pcl::PointXYZRGB>);
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr big_cloud_before_mv (new pcl::PointCloud<pcl::PointXYZRGB>);
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr big_cloud_after_mv (new pcl::PointCloud<pcl::PointXYZRGB>);
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr big_cloud_points_used_for_mv (new pcl::PointCloud<pcl::PointXYZRGB>);
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr big_cloud_points_used_for_mv_before (new pcl::PointCloud<pcl::PointXYZRGB>);

                for(size_t i=0; i < object_clouds.size(); i++)
                {
                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr trans_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
                    pcl::transformPointCloud(*scene_clouds[i], *trans_cloud, transformations[i]);
                    *big_cloud_after_mv += *trans_cloud;
                    *big_cloud_before_mv += *scene_clouds[i];

                    {
                        pcl::PointCloud<pcl::PointXYZRGB>::Ptr trans_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
                        pcl::transformPointCloud(*object_clouds[i], *trans_cloud, transformations[i]);
                        *big_cloud_points_used_for_mv += *trans_cloud;
                    }

                    {
                        *big_cloud_points_used_for_mv_before += *object_clouds[i];
                    }

                    {
                        pcl::PointCloud<pcl::PointXYZRGB>::Ptr trans_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
                        pcl::transformPointCloud(*scene_clouds_no_filter[i], *trans_cloud, transformations[i]);
                        *big_cloud_after_mv_unfiltered += *trans_cloud;
                    }
                }

                pcl::PointCloud<pcl::PointXYZRGB>::Ptr big_cloud_vx_after_mv (new pcl::PointCloud<pcl::PointXYZRGB>);

                {
                    std::cout << "Going to voxelgrid registered scene" << std::endl;
                    faat_pcl::utils::miscellaneous::voxelGridWithOctree(big_cloud_after_mv, *big_cloud_vx_after_mv, leaf);
                    std::cout << "Finished voxelgrid..." << std::endl;
                }

                pcl::visualization::PCLVisualizer vis ("registered cloud");
                int v1, v2, v3;
                vis.createViewPort (0, 0, 0.33, 1, v1);
                vis.createViewPort (0.33, 0, 0.66, 1, v2);
                vis.createViewPort (0.66, 0, 1, 1, v3);
                vis.setBackgroundColor(1,1,1);

                //pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> handler (big_cloud_points_used_for_mv);
                //vis.addPointCloud (big_cloud_points_used_for_mv, handler, "big", v1);

                pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> handler (big_cloud_after_mv_unfiltered);
                vis.addPointCloud (big_cloud_after_mv_unfiltered, handler, "big", v1);

                {
                    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> handler (big_cloud_after_mv);
                    vis.addPointCloud (big_cloud_after_mv, handler, "big_mv", v2);
                }

                //{
                //    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> handler (big_cloud_points_used_for_mv_before);
                //    vis.addPointCloud (big_cloud_points_used_for_mv_before, handler, "big_mv_before", v2);
                //}

                //{
                //    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> handler (big_cloud_points_used_for_mv);
                //    vis.addPointCloud (big_cloud_points_used_for_mv, handler, "keypoints_mv", v3);
                ///

                {
                    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> handler (big_cloud_before_mv);
                    vis.addPointCloud (big_cloud_before_mv, handler, "keypoints_mv", v3);
                }

                vis.spin ();
            }*/

            //visualize the model hypotheses
            std::vector<pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr> aligned_models;
            std::vector < std::string > ids;
            std::vector < Eigen::Matrix4f > transforms_to_global;
            std::vector< Eigen::Matrix4f > hypotheses_poses_in_global_frame;
            std::vector<typename pcl::PointCloud<pcl::Normal>::ConstPtr> aligned_normals;

            boost::shared_ptr < faat_pcl::rec_3d_framework::ModelOnlySource<pcl::PointXYZRGBNormal, pcl::PointXYZRGB>
                    > source (new faat_pcl::rec_3d_framework::ModelOnlySource<pcl::PointXYZRGBNormal, pcl::PointXYZRGB>);
            source->setPath ( models_dir_ );
            source->setLoadViews (false);
            source->setLoadIntoMemory(false);
            std::string test = "irrelevant";
            source->generate (test);
            source->createVoxelGridAndDistanceTransform (go3d_and_icp_resolution_);

            std::vector<int> hyp_index_to_vp;
            int kk=0;
            for (vp = vertices (grph_final_); vp.first != vp.second; ++vp.first, kk++)
            {
                std::cout << *vp.first << " " << kk << std::endl;
                //transforms_to_global.push_back (transformations[kk] * grph_final[*vp.first].absolute_pose);
                transforms_to_global.push_back (grph_final_[*vp.first].absolute_pose);

                for (std::vector<Hypothesis>::iterator it_hyp = grph_final_[*vp.first].hypothesis.begin (); it_hyp != grph_final_[*vp.first].hypothesis.end (); ++it_hyp)
                {
                    if(it_hyp->extended_)
                        continue;

                    std::vector < std::string > strs_2;
                    boost::split (strs_2, it_hyp->model_id_, boost::is_any_of ("/\\"));
                    ModelTPtr model;
                    source->getModelById (strs_2[strs_2.size () - 1], model);

                    //Eigen::Matrix4f trans = transformations[kk] * grph_final[*vp.first].absolute_pose * it_hyp->transform_;
                    Eigen::Matrix4f trans = grph_final_[*vp.first].absolute_pose * it_hyp->transform_;
                    ConstPointInTPtr model_cloud = model->getAssembled (go3d_and_icp_resolution_);
                    pcl::PointCloud<pcl::Normal>::ConstPtr normal_cloud = model->getNormalsAssembled (go3d_and_icp_resolution_);

                    typename pcl::PointCloud<pcl::Normal>::Ptr normal_aligned (new pcl::PointCloud<pcl::Normal>(*normal_cloud));
                    typename pcl::PointCloud<PointT>::Ptr model_aligned (new pcl::PointCloud<PointT>(*model_cloud));

                    hypotheses_poses_in_global_frame.push_back(trans);
                    aligned_models.push_back (model_aligned);
                    aligned_normals.push_back(normal_aligned);
                    ids.push_back (it_hyp->model_id_);
                    hyp_index_to_vp.push_back(*vp.first);
                }

                if(use_unverified_single_view_hypotheses)
                {
                    std::cout << "use_unverified_single_view_hypotheses is true " << grph_final_[*vp.first].hypothesis_single_unverified.size() <<  std::endl;
                    for (std::vector<Hypothesis>::iterator it_hyp = grph_final_[*vp.first].hypothesis_single_unverified.begin ();
                         it_hyp != grph_final_[*vp.first].hypothesis_single_unverified.end (); ++it_hyp)
                    {
                        if(it_hyp->extended_)
                            continue;

                        std::vector < std::string > strs_2;
                        boost::split (strs_2, ids[kk], boost::is_any_of ("/\\"));
                        ModelTPtr model;
                        bool found = source->getModelById (strs_2[strs_2.size () - 1], model);

                        //Eigen::Matrix4f trans = transformations[kk] * grph_final[*vp.first].absolute_pose * it_hyp->transform_;
                        Eigen::Matrix4f trans = grph_final_[*vp.first].absolute_pose * it_hyp->transform_;
                        ConstPointInTPtr model_cloud = model->getAssembled (go3d_and_icp_resolution_);
                        pcl::PointCloud<pcl::Normal>::ConstPtr normal_cloud = model->getNormalsAssembled (go3d_and_icp_resolution_);

                        typename pcl::PointCloud<pcl::Normal>::Ptr normal_aligned (new pcl::PointCloud<pcl::Normal>(*normal_cloud));
                        typename pcl::PointCloud<PointT>::Ptr model_aligned (new pcl::PointCloud<PointT>(*model_cloud));

                        hypotheses_poses_in_global_frame.push_back(trans);
                        aligned_models.push_back (model_aligned);
                        aligned_normals.push_back(normal_aligned);
                        ids.push_back (it_hyp->model_id_);
                        hyp_index_to_vp.push_back(*vp.first);
                    }
                }
                else
                {
                    std::cout << "use_unverified_single_view_hypotheses is false" << std::endl;
                }
            }

            std::cout << "number of hypotheses for GO3D:" << aligned_models.size() << std::endl;
            if(aligned_models.size() > 0)
            {

                pcl::PointCloud<pcl::PointXYZRGB>::Ptr big_cloud_go3D(new pcl::PointCloud<pcl::PointXYZRGB>);
                pcl::PointCloud<pcl::Normal>::Ptr big_cloud_go3D_normals(new pcl::PointCloud<pcl::Normal>);

                {
                    //obtain big cloud and occlusion clouds based on new noise model integration
                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr octree_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
                    faat_pcl::utils::NMBasedCloudIntegration<pcl::PointXYZRGB> nmIntegration;
                    nmIntegration.setInputClouds(original_clouds);
                    nmIntegration.setResolution(go3d_and_icp_resolution_);
                    nmIntegration.setWeights(views_noise_weights);
                    nmIntegration.setTransformations(transforms_to_global);
                    nmIntegration.setMinWeight(nm_integration_min_weight_);
                    nmIntegration.setInputNormals(normal_clouds);
                    nmIntegration.setMinPointsPerVoxel(1);
                    nmIntegration.setFinalResolution(go3d_and_icp_resolution_);
                    nmIntegration.compute(octree_cloud);

                    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> used_clouds;
                    pcl::PointCloud<pcl::Normal>::Ptr big_normals(new pcl::PointCloud<pcl::Normal>);
                    nmIntegration.getOutputNormals(big_normals);
                    nmIntegration.getInputCloudsUsed(used_clouds);

                    occlusion_clouds.resize(used_clouds.size());
                    for(size_t kk=0; kk < used_clouds.size(); kk++)
                    {
                        occlusion_clouds[kk].reset(new pcl::PointCloud<pcl::PointXYZRGB>(*used_clouds[kk]));
                    }

                    big_cloud_go3D = octree_cloud;
                    big_cloud_go3D_normals = big_normals;
                }

                //Refine aligned models with ICP
                if(go3d_icp_)
                {
                    pcl::ScopeTime t("GO3D ICP...\n");
                    float icp_max_correspondence_distance_ = 0.01f;

#pragma omp parallel for num_threads(4) schedule(dynamic)
                    for(size_t kk=0; kk < aligned_models.size(); kk++)
                    {

                        std::vector < std::string > strs_2;
                        boost::split (strs_2, ids[kk], boost::is_any_of ("/\\"));
                        ModelTPtr model;
                        source->getModelById (strs_2[strs_2.size () - 1], model);

                        //cut scene based on model cloud
                        boost::shared_ptr < distance_field::PropagationDistanceField<pcl::PointXYZRGB> > dt;
                        model->getVGDT (dt);

                        pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud;
                        dt->getInputCloud (cloud);

                        Eigen::Matrix4f scene_to_model_trans = hypotheses_poses_in_global_frame[kk].inverse ();
                        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_voxelized_icp_transformed (new pcl::PointCloud<pcl::PointXYZRGB> ());
                        pcl::transformPointCloud (*big_cloud_go3D, *cloud_voxelized_icp_transformed, scene_to_model_trans);

                        float thres = icp_max_correspondence_distance_ * 2.f;
                        pcl::PointXYZRGB minPoint, maxPoint;
                        pcl::getMinMax3D(*cloud, minPoint, maxPoint);
                        minPoint.x -= thres;
                        minPoint.y -= thres;
                        minPoint.z -= thres;

                        maxPoint.x += thres;
                        maxPoint.y += thres;
                        maxPoint.z += thres;

                        pcl::CropBox<pcl::PointXYZRGB> cropFilter;
                        cropFilter.setInputCloud (cloud_voxelized_icp_transformed);
                        cropFilter.setMin(minPoint.getVector4fMap());
                        cropFilter.setMax(maxPoint.getVector4fMap());

                        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_voxelized_icp_cropped (new pcl::PointCloud<pcl::PointXYZRGB> ());
                        cropFilter.filter (*cloud_voxelized_icp_cropped);

                        if(go3d_icp_model_to_scene_)
                        {
                            Eigen::Matrix4f s2m = scene_to_model_trans.inverse();
                            pcl::transformPointCloud (*cloud_voxelized_icp_cropped, *cloud_voxelized_icp_cropped, s2m);

                            pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
                            icp.setInputTarget (cloud_voxelized_icp_cropped);
                            icp.setInputSource(aligned_models[kk]);
                            icp.setMaxCorrespondenceDistance(icp_max_correspondence_distance_);
                            icp.setMaximumIterations(icp_iter_);
                            icp.setRANSACIterations(5000);
                            icp.setEuclideanFitnessEpsilon(1e-12);
                            icp.setTransformationEpsilon(1e-12);
                            pcl::PointCloud < PointT >::Ptr model_aligned( new pcl::PointCloud<PointT> );
                            icp.align (*model_aligned, hypotheses_poses_in_global_frame[kk]);

                            hypotheses_poses_in_global_frame[kk] = icp.getFinalTransformation();
                        }
                        else
                        {
                            faat_pcl::rec_3d_framework::VoxelBasedCorrespondenceEstimation<pcl::PointXYZRGB, pcl::PointXYZRGB>::Ptr
                                    est (
                                        new faat_pcl::rec_3d_framework::VoxelBasedCorrespondenceEstimation<
                                        pcl::PointXYZRGB,
                                        pcl::PointXYZRGB> ());

                            pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZRGB>::Ptr
                                    rej (
                                        new pcl::registration::CorrespondenceRejectorSampleConsensus<
                                        pcl::PointXYZRGB> ());

                            est->setVoxelRepresentationTarget (dt);
                            est->setInputSource (cloud_voxelized_icp_cropped);
                            est->setInputTarget (cloud);
                            est->setMaxCorrespondenceDistance (icp_max_correspondence_distance_);
                            est->setMaxColorDistance (-1, -1);

                            rej->setInputTarget (cloud);
                            rej->setMaximumIterations (5000);
                            rej->setInlierThreshold (icp_max_correspondence_distance_);
                            rej->setInputSource (cloud_voxelized_icp_cropped);

                            pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> reg;
                            reg.setCorrespondenceEstimation (est);
                            reg.addCorrespondenceRejector (rej);
                            reg.setInputTarget (cloud); //model
                            reg.setInputSource (cloud_voxelized_icp_cropped); //scene
                            reg.setMaximumIterations (icp_iter_);
                            reg.setEuclideanFitnessEpsilon (1e-12);
                            reg.setTransformationEpsilon (0.0001f * 0.0001f);

                            pcl::registration::DefaultConvergenceCriteria<float>::Ptr convergence_criteria;
                            convergence_criteria = reg.getConvergeCriteria ();
                            convergence_criteria->setAbsoluteMSE (1e-12);
                            convergence_criteria->setMaximumIterationsSimilarTransforms (15);
                            convergence_criteria->setFailureAfterMaximumIterations (false);

                            PointInTPtr output (new pcl::PointCloud<pcl::PointXYZRGB> ());
                            reg.align (*output);
                            Eigen::Matrix4f trans, icp_trans;
                            trans = reg.getFinalTransformation () * scene_to_model_trans;
                            icp_trans = trans.inverse ();

                            hypotheses_poses_in_global_frame[kk] = icp_trans;
                        }
                    }
                }

                //transform models to be used during GO3D
#pragma omp parallel for num_threads(4) schedule(dynamic)
                for(size_t kk=0; kk < aligned_models.size(); kk++)
                {
                    pcl::PointCloud < PointT >::Ptr model_aligned( new pcl::PointCloud<PointT> );
                    pcl::transformPointCloud(*aligned_models[kk], *model_aligned, hypotheses_poses_in_global_frame[kk]);
                    aligned_models[kk] = model_aligned;

                    typename pcl::PointCloud<pcl::Normal>::Ptr normal_aligned (new pcl::PointCloud<pcl::Normal>);
                    faat_pcl::utils::miscellaneous::transformNormals(aligned_normals[kk], normal_aligned, hypotheses_poses_in_global_frame[kk]);
                    aligned_normals[kk] = normal_aligned;
                }

                //Instantiate HV go 3D, reimplement addModels that will reason about occlusions
                //Set occlusion cloudS!!
                //Set the absolute poses so we can go from the global coordinate system to the occlusion clouds
                //TODO: Normals might be a problem!! We need normals from the models and normals from the scene, correctly oriented!
                //right now, all normals from the scene will be oriented towards some weird 0, same for models actually

                /*eps_angle_threshold_ = 0.25;
                min_points_ = 20;
                curvature_threshold_ = 0.04f;
                cluster_tolerance_ = 0.015f;
                setSmoothSegParameters (float t_eps, float curv_t, float dist_t, int min_points = 20)*/

                std::cout << "GO 3D parameters:" << std::endl;
                std::cout << "go3d_inlier_threshold_:" << go3d_inlier_threshold_ << std::endl;
                std::cout << "go3d_clutter_radius_:" << go3d_clutter_radius_ << std::endl;
                std::cout << "go3d_outlier_regularizer:" << go3d_outlier_regularizer_ << std::endl;
                std::cout << "go3d_clutter_regularizer:" << go3d_clutter_regularizer_ << std::endl;
                std::cout << "go3d_use_supervoxels:" << go3d_use_supervoxels_ << std::endl;
                std::cout << "go3d_color_sigma:" << go3d_color_sigma_ << std::endl;

                faat_pcl::GO3D<PointT, PointT> go;
                go.setResolution (go3d_and_icp_resolution_);
                go.setAbsolutePoses (transforms_to_global);
                go.setSmoothSegParameters(0.1, 0.04f, 0.01f, 100);
                go.setUseSuperVoxels(go3d_use_supervoxels_);
                go.setOcclusionsClouds (occlusion_clouds);
                go.setZBufferSelfOcclusionResolution (250);
                go.setInlierThreshold (go3d_inlier_threshold_);
                go.setRadiusClutter (go3d_clutter_radius_);
                go.setDetectClutter (go3d_detect_clutter_); //Attention, detect clutter turned off!
                go.setRegularizer (go3d_outlier_regularizer_);
                go.setClutterRegularizer (go3d_clutter_regularizer_);
                go.setHypPenalty (0.05f);
                go.setIgnoreColor (false);
                go.setColorSigma (go3d_color_sigma_);
                go.setOptimizerType (opt_type_);
                go.setDuplicityCMWeight(0.f);
                go.setSceneCloud (big_cloud_go3D);
                go.setSceneAndNormals(big_cloud_go3D, big_cloud_go3D_normals);
                go.setRequiresNormals(true);
                go.addNormalsClouds(aligned_normals);
                go.addModels (aligned_models, true);

                std::vector<faat_pcl::PlaneModel<PointT> > planes_found;

                /*if(go3d_add_planes)
                {
                    faat_pcl::MultiPlaneSegmentation<PointT> mps;
                    mps.setInputCloud(big_cloud_after_mv);
                    mps.setMinPlaneInliers(5000);
                    mps.setResolution(leaf);
                    //mps.setNormals(grph[vrtx].normals_);
                    mps.setMergePlanes(false);
                    mps.segment();
                    planes_found = mps.getModels();

                    go.addPlanarModels(planes_found);
                    for(size_t kk=0; kk < planes_found.size(); kk++)
                    {
                        std::stringstream plane_id;
                        plane_id << "plane_" << kk;
                        ids.push_back(plane_id.str());
                    }
                }*/

                go.setObjectIds (ids);

                go.verify ();
                std::vector<bool> mask;
                go.getMask (mask);

                if(visualize_output_)
                {
                    pcl::visualization::PCLVisualizer vis ("registered cloud");
                    int v1, v2, v3, v4, v5, v6;
                    vis.createViewPort (0, 0, 0.5, 0.33, v1);
                    vis.createViewPort (0.5, 0, 1, 0.33, v2);
                    vis.createViewPort (0, 0.33, 0.5, 0.66, v3);
                    vis.createViewPort (0.5, 0.33, 1, 0.66, v4);
                    vis.createViewPort (0, 0.66, 0.5, 1, v5);
                    vis.createViewPort (0.5, 0.66, 1, 1, v6);

                    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> handler (big_cloud_go3D);
                    vis.addPointCloud (big_cloud_go3D, handler, "big", v1);

                    /*pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> handler (big_cloud_vx_after_mv);
            vis.addPointCloud (big_cloud_vx_after_mv, handler, "big", v1);*/

                    for(size_t i=0; i < aligned_models.size(); i++)
                    {
                        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> handler_rgb_verified (aligned_models[i]);
                        std::stringstream name;
                        name << "Hypothesis_model_" << i;
                        vis.addPointCloud<pcl::PointXYZRGB> (aligned_models[i], handler_rgb_verified, name.str (), v2);
                    }

                    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr smooth_cloud_ =  go.getSmoothClustersRGBCloud();
                    if(smooth_cloud_)
                    {
                        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> random_handler (smooth_cloud_);
                        vis.addPointCloud<pcl::PointXYZRGBA> (smooth_cloud_, random_handler, "smooth_cloud", v5);
                    }

                    for (size_t i = 0; i < aligned_models.size (); i++)
                    {
                        if (mask[i])
                        {
                            std::cout << "Verified:" << ids[i] << std::endl;
                            pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> handler_rgb_verified (aligned_models[i]);
                            std::stringstream name;
                            name << "verified" << i;
                            vis.addPointCloud<pcl::PointXYZRGB> (aligned_models[i], handler_rgb_verified, name.str (), v3);

                            pcl::PointCloud<pcl::PointXYZRGB>::Ptr inliers_outlier_cloud;
                            go.getInlierOutliersCloud((int)i, inliers_outlier_cloud);

                            {
                                pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> handler_rgb_verified (inliers_outlier_cloud);
                                std::stringstream name;
                                name << "verified_visible_" << i;
                                vis.addPointCloud<pcl::PointXYZRGB> (inliers_outlier_cloud, handler_rgb_verified, name.str (), v4);
                            }
                        }
                    }

                    if(go3d_add_planes_)
                    {
                        for(size_t i=0; i < planes_found.size(); i++)
                        {
                            if(!mask[i + aligned_models.size()])
                                continue;

                            std::stringstream pname;
                            pname << "plane_" << i;

                            pcl::visualization::PointCloudColorHandlerRandom<PointT> scene_handler(planes_found[i].plane_cloud_);
                            vis.addPointCloud<PointT> (planes_found[i].plane_cloud_, scene_handler, pname.str(), v3);

                            //pname << "chull";
                            //vis.addPolygonMesh (planes_found[i].convex_hull_, pname.str(), v3);
                        }
                    }

                    vis.setBackgroundColor(1,1,1);
                    vis.spin ();
                }

                for (size_t i = 0; i < aligned_models.size (); i++)
                {
                    if (mask[i])
                    {
                        int k=0;
                        for (vp = vertices (grph_final_); vp.first != vp.second; ++vp.first, k++)
                        {
                            //hypotheses_poses_in_global_frame[i] transforms from object coordinates to global coordinate system
                            //transforms_to_global aligns a single frame to the global coordinate system
                            //transformation would then be a transformation transforming first the object to global coordinate system
                            //concatenated with the inverse of transforms_to_global[k]
                            Eigen::Matrix4f t = transforms_to_global[k].inverse() * hypotheses_poses_in_global_frame[i];
                            std::string origin = "3d go";
                            Hypothesis hyp(ids[i], t, origin, true, true);
                            grph_final_[*vp.first].hypothesis.push_back(hyp);
                        }
                    }
                }

                if(output_dir_3d_results_.compare("") != 0)
                {
                    bf::path out_dir_path = output_dir_3d_results_;
                    if(!bf::exists(out_dir_path))
                    {
                        bf::create_directory(out_dir_path);
                    }

                    for(size_t i=0; i < occlusion_clouds.size(); i++)
                    {
                        std::stringstream pose_path;
                        pose_path << output_dir_3d_results_ << "/transformation_" << setw(5) << setfill('0') << i << ".txt";
                        faat_pcl::rec_3d_framework::PersistenceUtils::writeMatrixToFile(pose_path.str(), transforms_to_global[i]);

                        std::stringstream cloud_path;
                        cloud_path << output_dir_3d_results_ << "/cloud_" << setw(5) << setfill('0') << i << ".pcd";
                        pcl::io::savePCDFileBinary(cloud_path.str(), *occlusion_clouds[i]);

                        {
                            {
                                std::stringstream cloud_path;
                                cloud_path << output_dir_3d_results_ << "/original_clouds/";
                                bf::path out_dir_path = cloud_path.str();
                                if(!bf::exists(out_dir_path))
                                {
                                    bf::create_directory(out_dir_path);
                                }
                            }

                            std::stringstream cloud_path;
                            cloud_path << output_dir_3d_results_ << "/original_clouds/cloud_" << setw(5) << setfill('0') << i << ".pcd";
                            pcl::io::savePCDFileBinary(cloud_path.str(), *original_clouds[i]);
                        }
                    }

                    std::stringstream results_path;
                    results_path << output_dir_3d_results_ << "/results_3d.txt";

                    std::ofstream out (results_path.str ().c_str());
                    if (!out)
                    {
                        std::cout << "Cannot open file.\n";
                    }

                    for (size_t k = 0; k < aligned_models.size (); k++)
                    {
                        if (mask[k])
                        {
                            out << ids[k] << "\t";
                            for (size_t i = 0; i < 4; i++)
                            {
                                for (size_t j = 0; j < 4; j++)
                                {
                                    out << hypotheses_poses_in_global_frame[k] (i, j);
                                    if (!(i == 3 && j == 3))
                                        out << "\t";
                                }
                            }

                            out << std::endl;
                        }
                    }

                    out.close ();
                }
            }
        }
        else
        {
            //---Verify-extended-hypotheses-and-visualize------------------------
            bool current_iteration_done = false;

            std::pair<vertex_iter, vertex_iter> vp;
            for ( vp = vertices ( grph_final_ ); (vp.first != vp.second) && (!current_iteration_done); ++vp.first )
            {

                Vertex vrtx_tmp;

                if ( ! do_reverse_hyp_extension)	// It only applies ICP on the latest scene point cloud
                {
                    vrtx_tmp = vrtx_final;
                    current_iteration_done = true;
                }
                else
                {
                    vrtx_tmp = *vp.first;
                }

                std::vector<pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr> aligned_models;
                std::vector < std::string > ids;

                for ( std::vector<Hypothesis>::iterator it_hyp = grph_final_[vrtx_tmp].hypothesis.begin (); it_hyp != grph_final_[vrtx_tmp].hypothesis.end (); ++it_hyp )
                {
                    PointInTPtr pModelPCl ( new pcl::PointCloud<pcl::PointXYZRGB> );
                    PointInTPtr pModelPClTransformed ( new pcl::PointCloud<pcl::PointXYZRGB> );
                    PointInTPtr pModelPCl2 ( new pcl::PointCloud<pcl::PointXYZRGB> );
                    pcl::io::loadPCDFile ( it_hyp->model_id_, * ( pModelPCl ) );

                    pcl::transformPointCloud ( *pModelPCl, *pModelPClTransformed, it_hyp->transform_ );

                    pcl::VoxelGrid<pcl::PointXYZRGB> sor;
                    float leaf = 0.005f;
                    sor.setLeafSize ( leaf, leaf, leaf );
                    sor.setInputCloud ( pModelPClTransformed );
                    sor.filter ( *pModelPCl2 );

                    aligned_models.push_back ( pModelPCl2 );
                    ids.push_back ( it_hyp->model_id_ );
                }
                std::cout << "View " << grph_final_[vrtx_tmp].view_id_ << " has " << grph_final_[vrtx_tmp].hypothesis.size ()
                          << " hypothesis. " << std::endl;

                //initialize go
                float go_resolution_ = 0.005f;
                bool add_planes = true;
                float assembled_resolution = 0.003f;
                float color_sigma = 0.5f;

                boost::shared_ptr<faat_pcl::GlobalHypothesesVerification_1<PointT, PointT> > go (
                            new faat_pcl::GlobalHypothesesVerification_1<PointT,
                            PointT> );

                go->setSmoothSegParameters ( 0.1, 0.035, 0.005 );
                //go->setRadiusNormals(0.03f);
                go->setResolution ( go_resolution_ );
                go->setInlierThreshold ( 0.01 );
                go->setRadiusClutter ( 0.03f );
                go->setRegularizer ( 2 );
                go->setClutterRegularizer ( 5 );
                go->setDetectClutter ( true );
                go->setOcclusionThreshold ( 0.01f );
                go->setOptimizerType ( 0 );
                go->setUseReplaceMoves ( true );
                go->setRadiusNormals ( 0.02 );
                go->setRequiresNormals ( false );
                go->setInitialStatus ( false );
                go->setIgnoreColor ( false );
                go->setColorSigma ( color_sigma );
                go->setUseSuperVoxels ( false );


                //Multiplane segmentation
                faat_pcl::MultiPlaneSegmentation<PointT> mps;
                mps.setInputCloud ( grph_final_[vrtx_tmp].pScenePCl );
                mps.setMinPlaneInliers ( 1000 );
                mps.setResolution ( go_resolution_ );
                mps.setNormals ( grph_final_[vrtx_tmp].pSceneNormals );
                mps.setMergePlanes ( true );
                std::vector<faat_pcl::PlaneModel<PointT> > planes_found;
                mps.segment();
                planes_found = mps.getModels();

                if ( planes_found.size() == 0 && grph_final_[vrtx_tmp].pScenePCl->isOrganized() )
                {
                    PCL_WARN ( "No planes found, doing segmentation with standard method\n" );
                    mps.segment ( true );
                    planes_found = mps.getModels();
                }

                std::vector<pcl::PointIndices> indices;
                Eigen::Vector4f table_plane;
                doSegmentation<PointT> ( grph_final_[vrtx_tmp].pScenePCl, grph_final_[vrtx_tmp].pSceneNormals, indices, table_plane );

                std::vector<int> indices_above_plane;
                for ( int k = 0; k < grph_final_[vrtx_tmp].pScenePCl->points.size (); k++ )
                {
                    Eigen::Vector3f xyz_p = grph_final_[vrtx_tmp].pScenePCl->points[k].getVector3fMap ();
                    if ( !pcl_isfinite ( xyz_p[0] ) || !pcl_isfinite ( xyz_p[1] ) || !pcl_isfinite ( xyz_p[2] ) )
                        continue;

                    float val = xyz_p[0] * table_plane[0] + xyz_p[1] * table_plane[1] + xyz_p[2] * table_plane[2] + table_plane[3];
                    if ( val >= 0.01 )
                        indices_above_plane.push_back ( static_cast<int> ( k ) );
                }

                std::vector<std::string> model_ids;
                typename pcl::PointCloud<PointT>::Ptr occlusion_cloud ( new pcl::PointCloud<PointT> ( *grph_final_[vrtx_tmp].pScenePCl ) );
                go->setSceneCloud ( grph_final_[vrtx_tmp].pScenePCl );
                go->setNormalsForClutterTerm ( grph_final_[vrtx_tmp].pSceneNormals );
                go->setOcclusionCloud ( occlusion_cloud );
                //addModels
                go->addModels ( aligned_models, true );
                //append planar models
                if ( add_planes )
                {
                    go->addPlanarModels ( planes_found );
                    for ( size_t kk=0; kk < planes_found.size(); kk++ )
                    {
                        std::stringstream plane_id;
                        plane_id << "plane_" << kk;
                        model_ids.push_back ( plane_id.str() );
                    }
                }

                go->setObjectIds ( model_ids );
                //verify
                {
                    pcl::ScopeTime t ( "Go verify" );
                    go->verify ();
                }
                std::vector<bool> mask_hv;
                go->getMask ( mask_hv );


                for ( size_t hyp_id = 0; hyp_id < aligned_models.size(); hyp_id++ )
                {
                    std::cout << hyp_id << "is" << static_cast<int> ( mask_hv[hyp_id] ) << std::endl;
                    //std::cout << static_cast<int> (mask_hv[j]) << std::endl;
                    if ( !mask_hv[hyp_id] )
                    {
                        grph_final_[vrtx_tmp].hypothesis[hyp_id].verified_ = false;
                    }
                    else
                    {
                        grph_final_[vrtx_tmp].hypothesis[hyp_id].verified_ = true;
//                        std_msgs::String ss;
//                        ss.data = grph_final_[vrtx_tmp].hypothesis[hyp_id].model_id_;
//                        response.ids.push_back(ss);

//                        Eigen::Matrix4f trans = grph_final_[vrtx_tmp].hypothesis[hyp_id].transform_;
//                        geometry_msgs::Transform tt;
//                        tt.translation.x = trans(0,3);
//                        tt.translation.y = trans(1,3);
//                        tt.translation.z = trans(2,3);

//                        Eigen::Matrix3f rotation = trans.block<3,3>(0,0);
//                        Eigen::Quaternionf q(rotation);
//                        tt.rotation.x = q.x();
//                        tt.rotation.y = q.y();
//                        tt.rotation.z = q.z();
//                        tt.rotation.w = q.w();
//                        response.transforms.push_back(tt);
                    }
                }
            }
        }
    }
    else
    {
        for ( size_t hyp_id = 0; hyp_id < grph_final_[vrtx_final].hypothesis.size(); hyp_id++ )
        {
            grph_[vrtx].hypothesis[hyp_id].verified_ = true;
            grph_final_[vrtx_final].hypothesis[hyp_id].verified_ = true;
        }
    }

    outputgraph ( grph_final_, "Final_with_Hypothesis_extension.dot" );


    //-----------------Visualize Scene Cloud-and-Recognition-Results---------------------------
    //--(bottom: Scene; 2nd from bottom: Single-view-results; 2nd from top: transformed hypotheses; top: verified hypotheses coming from all views)--
    if ( visualize_output_ )
    {
        std::vector<int> viewportNr;
        vis_->removeAllPointClouds();
        viewportNr = faat_pcl::utils::visualization_framework ( vis_, num_vertices(grph_final_), 4 );

        std::pair<vertex_iter, vertex_iter> vp;
        int view_id = -1;
        for ( vp = vertices ( grph_final_ ); vp.first != vp.second; ++vp.first )
        {
            view_id++;
            pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> handler_rgb ( grph_final_[*vp.first].pScenePCl );
            std::stringstream cloud_name;
            cloud_name << "view_cloud_" << grph_final_[*vp.first].view_id_;
            vis_->addPointCloud<pcl::PointXYZRGB> ( grph_final_[*vp.first].pScenePCl, handler_rgb, cloud_name.str (), viewportNr[view_id * 4 + 0] );

            for ( size_t hyp_id = 0; hyp_id < grph_final_[*vp.first].hypothesis.size(); hyp_id++ )
            {
                //visualize models
                std::string model_id = grph_final_[*vp.first].hypothesis[hyp_id].model_id_;
                std::string origin = grph_final_[*vp.first].hypothesis[hyp_id].origin_;
                Eigen::Matrix4f trans = grph_final_[*vp.first].hypothesis[hyp_id].transform_;

                std::stringstream name;
                name << cloud_name.str() << "___hypothesis_" << hyp_id << "___origin_" << origin;

                // 		ModelTPtr m;

                // 		models_source_->getModelById(model_id, m);
                //
                // 		ConstPointInTPtr model_cloud = m->getAssembled (0.003f);
                typename pcl::PointCloud<PointT>::Ptr pModelPCl ( new pcl::PointCloud<PointT> );
                typename pcl::PointCloud<PointT>::Ptr model_aligned ( new pcl::PointCloud<PointT> );

                pcl::io::loadPCDFile ( model_id, *pModelPCl );
                pcl::transformPointCloud ( *pModelPCl, *model_aligned, trans );

                pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb_handler ( model_aligned );
                vis_->addPointCloud<PointT> ( model_aligned, rgb_handler, name.str (), viewportNr[view_id * 4 +2] );

                if ( grph_final_[*vp.first].hypothesis[hyp_id].origin_.compare ( grph_final_[*vp.first].view_id_ ) == 0 )	//--show-hypotheses-from-single-view
                {
                    name << "__extended";
                    pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb_handler2 ( model_aligned );
                    vis_->addPointCloud<PointT> ( model_aligned, rgb_handler, name.str (), viewportNr[view_id * 4 + 1] );
                }

                if ( grph_final_[*vp.first].hypothesis[hyp_id].verified_ )	//--show-verified-extended-hypotheses
                {
                    name << "__verified";
                    pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb_handler3 ( model_aligned );
                    vis_->addPointCloud<PointT> ( model_aligned, rgb_handler, name.str (), viewportNr[view_id * 4 + 3] );
                }

            }
        }
        vis_->spin ();
        //vis->getInteractorStyle()->saveScreenshot ( "singleview.png" );
    }


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
    if(num_vertices(grph_) > 2)
    {
        Vertex vrtxToKill = getFurthestVertex(grph_);

        std::vector<Edge> edges_to_be_removed;
        typename graph_traits<Graph>::out_edge_iterator out_i, out_end;
        for ( tie ( out_i, out_end ) = out_edges ( vrtxToKill, grph_ ); out_i != out_end; ++out_i )
        {
            edges_to_be_removed.push_back(*out_i);
        }
        typename graph_traits<Graph>::in_edge_iterator in_i, in_end;
        for ( tie ( in_i, in_end ) = in_edges ( vrtxToKill, grph_ ); in_i != in_end; ++in_i )
        {
            edges_to_be_removed.push_back(*in_i);
        }


        for(size_t remover_id = 0; remover_id < edges_to_be_removed.size(); remover_id++)
        {
            remove_edge(edges_to_be_removed[remover_id], grph_);
        }

        remove_vertex(vrtxToKill, grph_);

        Vertex vrtxToKill_final = getFurthestVertex(grph_final_);

        std::vector<Edge> edges_to_be_removed_final;

        for ( tie ( out_i, out_end ) = out_edges ( vrtxToKill_final, grph_final_ ); out_i != out_end; ++out_i )
        {
            edges_to_be_removed_final.push_back(*out_i);
        }
        for ( tie ( in_i, in_end ) = in_edges ( vrtxToKill_final, grph_final_ ); in_i != in_end; ++in_i )
        {
            edges_to_be_removed_final.push_back(*in_i);
        }

        for(size_t remover_id = 0; remover_id < edges_to_be_removed_final.size(); remover_id++)
        {
            remove_edge(edges_to_be_removed_final[remover_id], grph_final_);
        }

        remove_vertex(vrtxToKill_final, grph_final_);
        outputgraph ( grph_final_, "final_after_deleting_old_vertex.dot" );
        outputgraph ( grph_, "grph_after_deleting_old_vertex.dot" );
    }

    return true;
}
