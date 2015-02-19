/*
 * main.cpp
 *
 * Multiview Object Recognition
 * 06.April 2013
 * Author: Thomas Fäulhammer
 *
 *
 *Command: ./multi_view_gt -models_dir_sift /media/Data/willow_dataset/willow_dense_shot30_reduced/ -training_dir /media/Data/willow_dataset/trained_test/ -model_path /media/Data/willow_dataset/hannes_clouds2_reduced/ -input_cloud_dir /media/Data/willow_dataset/willow_test/T_07/ -hypothesis detectedObjects.txt
 *
 */

#include <iostream>
#include <string>
#include <sstream>
#include <vector>
#include <ctime>
#include <numeric>

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/prim_minimum_spanning_tree.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/iteration_macros.hpp>
#include <boost/graph/graphviz.hpp>
#include <boost/filesystem.hpp>
#include <boost/property_map/property_map.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <pcl/console/parse.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/octree/octree.h>
#include <pcl/point_cloud.h>
#include <pcl/recognition/hv/occlusion_reasoning.h>
#include <pcl/search/organized.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/interactor_style.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/visualization/point_cloud_handlers.h>
#include <pcl/registration/icp.h>
#include <omp.h>
#include "myGraphClasses.h"
#include "myRecognizer.h"
#include "functions.h"

#include <v4r/ORFramework/faat_3d_rec_framework_defines.h>
#include <v4r/ORFramework/sift_local_estimator.h>
#include <v4r/ORFramework/model_only_source.h>
#include <v4r/ORRecognition/ghv.h>
#include <v4r/ORRecognition/hv_go_3D.h>
#include <v4r/ORRegistration/fast_icp_with_gc.h>
//#include <faat_pcl/registration/mv_lm_icp.h>
#include <v4r/ORUtils/filesystem_utils.h>
#include <v4r/ORUtils/registration_utils.h>
#include <v4r/ORUtils/pcl_opencv.h>
#include <v4r/ORUtils/noise_models.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/crop_box.h>
#include <v4r/ORUtils/miscellaneous.h>
#include <v4r/ORFramework//multiplane_segmentation.h>
#include <v4r/ORUtils/noise_model_based_cloud_integration.h>

#include <ros/ros.h>

//std::cout << "Debug Message: I am in file " << __FILE__ << " at line " << __LINE__ << std::endl;

#define SUBWINDOWS_PER_VIEW 3
#define SUBWINDOWS_PER_VIEW_HT_FROM_FILE 4

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointInT;
typedef PointInT::ConstPtr ConstPointInTPtr;
typedef boost::shared_ptr<PointInT> PointInTPtr;
typedef pcl::PointXYZRGB PointT;
typedef pcl::Histogram<128> FeatureT;
typename boost::graph_traits<Graph>::out_edge_iterator out_edge_iter;

typedef pcl::PointCloud<PointT>::Ptr PointCloudPtr;
typedef const boost::shared_ptr<const pcl::PointCloud<pcl::PointCloud<PointT> > > PointCloudConstPtr;

extern void
outputgraph (Graph& map, const char* filename);

//./multi_view_gt -training_dir /home/aitor/data/willow_challenge_trained_multiview/ -model_path /home/aitor/data/willow_object_clouds/models_ml_new/ -input_cloud_dir /home/aitor/data/willow_test/T_07/ -do_sift 0 -do_ourcvfh 0 -edge_weight_by_projection 1 -hypothesis T_07.dot -go_3d 1

//./multi_view_gt -model_path /home/aitor/data/willow/models/ -training_input_structure /home/aitor/data/willow/recognizer_structure/ -training_dir_sift /home/aitor/data/willow/sift_trained/ -input_cloud_dir /home/aitor/data/willow_dataset/T_14_willow_dataset/ -do_sift 0 -do_ourcvfh 0 -edge_weight_by_projection 1 -hypothesis test_t14.dot -go_3d 1 -models_dir_sift /home/aitor/data/willow/models/ -visualize_output 1 -sort_files 1

//./multi_view_gt -model_path /media/DATA/models/nice_models/ -training_input_structure /media/DATA/models/recognition_structure/ -training_dir_sift /media/DATA/sift_trained_iros/ -input_cloud_dir /media/DATA/iros_datasets/set_00000/ -do_sift 0 -do_ourcvfh 0 -edge_weight_by_projection 1 -hypothesis test_set00000.dot -go_3d 1 -models_dir_sift /media/DATA/models/nice_models/ -visualize_output 1 -sort_files 1

int
main (int argc, char **argv)
{
    ros::init ( argc, argv, "multiview_object_recognizer_batch_node" );
    boost::shared_ptr<ros::NodeHandle> nh;
    nh.reset( new ros::NodeHandle ( "~" ) );

    Graph grph, grph_final;
    std::vector<Vertex> vertices_v;
    std::vector<Edge> edges;
    std::string models_dir_sift, model_path, training_dir, input_cloud_dir, hypothesis_file;
    bool do_ourcvfh = true;
    bool visualize_output = true;
    int seg = 0;
    int icp_iter = 10;
    int opt_type = 0;
    int nrFiles;
    bool go_3d = true;
    std::string new_sift_models_, training_input_structure_, training_dir_new_sift_;
    std::string gt_or_ouput_dir = "";
    int sift_knn = 5;

    /*new_sift_models_ = "/media/Data/willow_dataset/models_ml_new";
   training_input_structure_ = "/media/Data/willow_dataset/willow_structure";
   training_dir_new_sift_ = "/home/thomas/Projects/thomas.faeulhammer/data/test_reg_views_source";*/

    new_sift_models_ = "/home/aitor/data/willow_object_clouds/models_ml_final/";
    training_input_structure_ = "/home/aitor/data/willow_structure_final";
    training_dir_new_sift_ = "/home/aitor/data/willow_sift_trained_final/";

    bool sort_files = false;
    std::string sift_idx = "sift_flann.idx";
    bool use_table_plane = true;
    int cg_size_threshold = 3;
    double cg_size = 0.01f;
    int max_node_distance_ = -1;
    bool scene_to_scene = true;
    bool use_unverified_single_view_hypotheses = false;
    bool only_scene_to_scene = false;
    double z_dist = 3.f;
    bool use_gc_s2s = true;
    double max_overlap_bf = 0.75f;
    bool do_sift = true;

    //Multiview refinement parameters
    bool mv_icp_ = true;
    double max_keypoint_dist_mv_ = 2.5f;
    int mv_iterations = 5;
    double min_overlap_mv = 0.3f;
    int mv_keypoints = 0;
    double inlier_threshold = 0.003f;
    double max_corresp_dist = 0.01f;

    nh->getParam ( "mv_iterations", mv_iterations);
    nh->getParam ( "max_keypoint_dist_mv", max_keypoint_dist_mv_);
    nh->getParam ( "mv_icp", mv_icp_);
    nh->getParam ( "mv_keypoints", mv_keypoints);
    nh->getParam ( "min_overlap_mv", min_overlap_mv);
    nh->getParam ( "inlier_threshold", inlier_threshold);
    nh->getParam ( "max_corresp_dist", max_corresp_dist);

    //GO3D parameters
    double go3d_color_sigma = 0.3f;
    double go3d_outlier_regularizer = 3.f;
    double go3d_clutter_regularizer = 3.f;
    double go3d_clutter_radius_ = 0.04f;
    double go3d_inlier_threshold_ = 0.01f;

    bool go3d_detect_clutter = true;
    bool go3d_add_planes = false;
    bool go3d_icp_ = true;
    bool go3d_icp_model_to_scene_ = false;
    bool go3d_use_supervoxels = true;

    double go3d_and_icp_resolution_ = 0.005f;

    nh->getParam ( "go3d_use_supervoxels", go3d_use_supervoxels);
    nh->getParam ( "go3d_icp_model_to_scene", go3d_icp_model_to_scene_);
    nh->getParam ( "go3d_icp", go3d_icp_);
    nh->getParam ( "go3d_add_planes", go3d_add_planes);
    nh->getParam ( "go3d_detect_clutter", go3d_detect_clutter);

    nh->getParam ( "go3d_and_icp_resolution", go3d_and_icp_resolution_);
    nh->getParam ( "go3d_clutter_radius", go3d_clutter_radius_);
    nh->getParam ( "go3d_color_sigma", go3d_color_sigma);
    nh->getParam ( "go3d_outlier_regularizer", go3d_outlier_regularizer);
    nh->getParam ( "go3d_clutter_regularizer", go3d_clutter_regularizer);
    nh->getParam ( "go3d_inlier_threshold", go3d_inlier_threshold_);
    nh->getParam ( "go_3d", go_3d);

    //SHOT parameters
    std::string shot_idx = "shot_flann.idx";
    bool do_shot = false;
    std::string shot_training_dir = "/media/DATA/shot_iros_trained";

    nh->getParam ( "shot_idx", shot_idx);
    nh->getParam ( "do_shot", do_shot);
    nh->getParam ( "shot_training_dir", shot_training_dir);

    //GO parameters
    double go_resolution = 0.005f;
    double go_inlier_threshold = 0.01f;
    double go_radius_clutter = 0.035f;
    double go_regularizer = 3.f;
    double go_clutter_reguralizer = 5.f;
    double go_color_sigma = 0.3f;

    nh->getParam ( "go_resolution", go_resolution);
    nh->getParam ( "go_inlier_threshold", go_inlier_threshold);
    nh->getParam ( "go_radius_clutter", go_radius_clutter);
    nh->getParam ( "go_regularizer", go_regularizer);
    nh->getParam ( "go_clutter_reguralizer", go_clutter_reguralizer);
    nh->getParam ( "go_color_sigma", go_color_sigma);

    //Noise model parameters
    double max_angle = 70.f;
    double lateral_sigma = 0.0015f;
    double nm_integration_min_weight_ = 0.25f;

    nh->getParam ( "max_angle", max_angle);
    nh->getParam ( "lateral_sigma", lateral_sigma);
    nh->getParam ( "nm_integration_min_weight", nm_integration_min_weight_);

    //Other parameters
    std::string output_dir_3d_results = "";
    nh->getParam ( "output_dir_3d_results", output_dir_3d_results);

    bool visualize_output_single_view = false;
    nh->getParam ( "visualize_output_single_view", visualize_output_single_view);
    nh->getParam ( "max_overlap_bf", max_overlap_bf);
    nh->getParam ( "use_gc_s2s", use_gc_s2s);
    nh->getParam ( "z_dist", z_dist);
    nh->getParam ( "only_scene_to_scene", only_scene_to_scene);
    nh->getParam ( "sift_knn", sift_knn);
    nh->getParam ( "use_unverified_sv_hyp", use_unverified_single_view_hypotheses);
    nh->getParam ( "scene_to_scene", scene_to_scene);
    nh->getParam ( "max_node_distance", max_node_distance_);
    nh->getParam ( "CG_SIZE", cg_size_threshold);
    nh->getParam ( "CG_THRESHOLD", cg_size);
    nh->getParam ( "use_table_plane", use_table_plane);
    nh->getParam ( "sift_idx", sift_idx);
    nh->getParam ( "sort_files", sort_files);
    nh->getParam ( "training_dir_sift", training_dir_new_sift_);
    nh->getParam ( "training_input_structure", training_input_structure_);
    nh->getParam ( "training_dir", training_dir);
    //nh->getParam ( "models_dir_sift", new_sift_models_);
    nh->getParam ( "model_path", model_path);
    nh->getParam ( "input_cloud_dir", input_cloud_dir);
    nh->getParam ( "hypothesis", hypothesis_file);
    nh->getParam ( "do_ourcvfh", do_ourcvfh);
    nh->getParam ( "do_sift", do_sift);
    nh->getParam ( "seg", seg);
    nh->getParam ( "icp_iterations", icp_iter);
    nh->getParam ( "opt_type", opt_type);
    nh->getParam ( "visualize_output", visualize_output);
    nh->getParam ( "gt_or_output_dir", gt_or_ouput_dir);

    new_sift_models_ = model_path;

    if(max_node_distance_ != -1 && !sort_files)
    {
        PCL_ERROR("Max node distance is different than -1 but files order is unknown\n");
        return -1;
    }

    if(only_scene_to_scene && !scene_to_scene)
    {
        PCL_ERROR("Only scene to scene mode but scene_to_scene edges set to false!\n");
        return -1;
    }

    bool single_view_hypothesis_exist = false;
    bf::path hyp_file = hypothesis_file;
    if (!bf::exists (hyp_file)) //no hypothesis exist yet --> create
    {
        bf::path obj_name_path = input_cloud_dir;
        std::vector < std::string > files_intern;
        std::string start = "";
        std::string ext = std::string ("pcd");
        getFilesInDirect (obj_name_path, start, files_intern, ext);

        if(sort_files)
            std::sort(files_intern.begin(), files_intern.end());
        nrFiles = files_intern.size ();

        myRecognizer myRec = myRecognizer ();
        myRec.setNewSiftParameters (new_sift_models_, training_input_structure_, training_dir_new_sift_);
        myRec.setDoOURCVFH (do_ourcvfh);
        myRec.setIcpIterations (icp_iter);
        myRec.setSIFTFlannIdx(sift_idx);
        myRec.setUseTablePlane(use_table_plane);
        myRec.setSegmentation (seg);
        myRec.setCGParams(cg_size_threshold, cg_size);
        myRec.setSiftKnn(sift_knn);
        myRec.setDoSift(do_sift);
        myRec.setDoShot(do_shot);
        myRec.setSHOTFlannIdx(shot_idx);
        myRec.setSHOTParameters(shot_training_dir);
        myRec.init (models_dir_sift, model_path, training_dir_new_sift_);

        for (size_t filevec_id = 0; filevec_id < nrFiles; filevec_id++)
        {
            Vertex vrtx = boost::add_vertex (grph);
            vertices_v.push_back (vrtx);

            grph[vrtx].scene_filename = input_cloud_dir + files_intern[filevec_id];

            pcl::io::loadPCDFile (grph[vrtx].scene_filename, *(grph[vrtx].pScenePCl));

            pcl::NormalEstimationOMP<PointT, pcl::Normal> ne;
            ne.setRadiusSearch(0.02f);
            ne.setInputCloud (grph[vrtx].pScenePCl);
            ne.compute (*(grph[vrtx].normals_));

            //----recognize-models---------------------------------------------
            myRec.setSceneNormals(grph[vrtx].normals_);
            myRec.recognize (grph[vrtx].pScenePCl);
            boost::shared_ptr < std::vector<ModelTPtr> > models = myRec.getModels ();
            boost::shared_ptr < std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > > transforms = myRec.getTransforms ();

            if(models->size() == 0)
                continue;

            std::vector<pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr> aligned_models;
            std::vector < std::string > ids;
            aligned_models.resize (models->size ());
            ids.resize (models->size ());
            std::vector<typename pcl::PointCloud<pcl::Normal>::ConstPtr> aligned_normals;
            aligned_normals.resize (models->size ());

            for (size_t j = 0; j < models->size (); j++)
            {
                ConstPointInTPtr model_cloud = models->at (j)->getAssembled (0.003f);
                pcl::PointCloud<pcl::Normal>::ConstPtr normal_cloud = models->at (j)->getNormalsAssembled (0.003f);

                PointInTPtr destination (new pcl::PointCloud<pcl::PointXYZRGB> ()); //optimization possibility -- Aitor: can you rewrite your code to accept non-constant pointer?

                pcl::transformPointCloud (*model_cloud, *destination, transforms->at (j));
                aligned_models[j] = destination;
                ids[j] = models->at (j)->id_;

                std::stringstream model_name;
                model_name << model_path << models->at (j)->id_;

                Hypothesis hypothesis (model_name.str (), transforms->at (j), grph[vrtx].scene_filename);
                grph[vrtx].hypothesis_single_all.push_back (hypothesis);

                typename pcl::PointCloud<pcl::Normal>::Ptr normal_aligned (new pcl::PointCloud<pcl::Normal>);
                faat_pcl::utils::miscellaneous::transformNormals(normal_cloud, normal_aligned, transforms->at (j));
                aligned_normals[j] = normal_aligned;
            }

            faat_pcl::OrganizedEdgeBase<PointT, pcl::Label> oed;
            oed.setDepthDisconThreshold (0.02f); //at 1m, adapted linearly with depth
            oed.setMaxSearchNeighbors(100);
            oed.setEdgeType (faat_pcl::OrganizedEdgeBase<PointT, pcl::Label>::EDGELABEL_OCCLUDING
            | faat_pcl::OrganizedEdgeBase<pcl::PointXYZRGB, pcl::Label>::EDGELABEL_OCCLUDED
            | faat_pcl::OrganizedEdgeBase<pcl::PointXYZRGB, pcl::Label>::EDGELABEL_NAN_BOUNDARY
            );
            oed.setInputCloud (grph[vrtx].pScenePCl);

            pcl::PointCloud<pcl::Label>::Ptr labels (new pcl::PointCloud<pcl::Label>);
            std::vector<pcl::PointIndices> indices2;
            oed.compute (*labels, indices2);

            pcl::PointCloud<pcl::PointXYZ>::Ptr occ_edges_full(new pcl::PointCloud<pcl::PointXYZ>);
            occ_edges_full->points.resize(grph[vrtx].pScenePCl->points.size());
            occ_edges_full->width = grph[vrtx].pScenePCl->width;
            occ_edges_full->height = grph[vrtx].pScenePCl->height;
            occ_edges_full->is_dense = grph[vrtx].pScenePCl->is_dense;

            for(size_t ik=0; ik < occ_edges_full->points.size(); ik++)
            {
                occ_edges_full->points[ik].x =
                occ_edges_full->points[ik].y =
                occ_edges_full->points[ik].z = std::numeric_limits<float>::quiet_NaN();
            }

            for (size_t j = 0; j < indices2.size (); j++)
            {
              for (size_t i = 0; i < indices2[j].indices.size (); i++)
              {
                occ_edges_full->points[indices2[j].indices[i]].getVector3fMap() = grph[vrtx].pScenePCl->points[indices2[j].indices[i]].getVector3fMap();
              }
            }

            //---Verify hypothesis and visualize---------
            boost::shared_ptr<faat_pcl::GHV<PointT, PointT> > go (new faat_pcl::GHV<PointT, PointT>);
            go->setInitialStatus (false);
//            go->setUseConflictGraph (false);
            go->setIgnoreColor (false);
            go->setUseReplaceMoves (true);
            go->setZBufferSelfOcclusionResolution (250);
            go->setOcclusionThreshold (0.0075f);
            go->setSelfOcclusionsReasoning (false);
            go->setResolution (go_resolution);
            go->setInlierThreshold (go_inlier_threshold);
            go->setRadiusClutter (go_radius_clutter);
            go->setRegularizer (go_regularizer);
            go->setClutterRegularizer (go_clutter_reguralizer);
            go->setHypPenalty (0.f);
            go->setDetectClutter (true);
            go->setColorSigma (go_color_sigma);
            go->setOptimizerType (opt_type);
            go->setOcclusionCloud (grph[vrtx].pScenePCl);
            go->setSceneCloud (grph[vrtx].pScenePCl);
            go->setNormalsForClutterTerm(grph[vrtx].normals_);
//            go->setOcclusionEdges(occ_edges_full);
            go->setDuplicityCMWeight(0.f);
            go->setRequiresNormals(true);
            go->addNormalsClouds(aligned_normals);
            go->addModels (aligned_models, true);

            faat_pcl::MultiPlaneSegmentation<PointT> mps;
            mps.setInputCloud(grph[vrtx].pScenePCl);
            mps.setMinPlaneInliers(1000);
            mps.setResolution(go_resolution);
            mps.setNormals(grph[vrtx].normals_);
            mps.setMergePlanes(true);
            std::vector<faat_pcl::PlaneModel<PointT> > planes_found;
            mps.segment();
            planes_found = mps.getModels();

            /*if(planes_found.size() == 0 && scene->isOrganized())
            {
                PCL_WARN("No planes found, doing segmentation with standard method\n");
                mps.segment(true);
                planes_found = mps.getModels();
            }*/

            go->addPlanarModels(planes_found);
            for(size_t kk=0; kk < planes_found.size(); kk++)
            {
                std::stringstream plane_id;
                plane_id << "plane_" << kk;
                ids.push_back(plane_id.str());
            }

            go->setObjectIds (ids);
            go->verify ();
            std::vector<bool> mask_hv;
            go->getMask (mask_hv);

            for (size_t j = 0; j < models->size (); j++)
            {

                std::stringstream model_name;
                model_name << model_path << models->at (j)->id_;
                Hypothesis hypothesis (model_name.str (), transforms->at (j), grph[vrtx].scene_filename, false);
                //std::cout << static_cast<int> (mask_hv[j]) << std::endl;
                if (!mask_hv[j])
                {
                    //save in unverified
                    grph[vrtx].hypothesis_single_unverified.push_back (hypothesis);
                    continue;
                }


                grph[vrtx].hypothesis.push_back (hypothesis);
            }

            int verified_planes = 0;
            for(size_t j=0; j < planes_found.size(); j++)
            {
                if(mask_hv[models->size() + j])
                {
                    grph[vrtx].verified_planes_.push_back(planes_found[j]);
                    verified_planes++;
                }
            }

            std::cout << "verified planes:" << verified_planes << std::endl;

        }

        if(visualize_output_single_view)
        {

            boost::shared_ptr < faat_pcl::rec_3d_framework::ModelOnlySource<pcl::PointXYZRGBNormal, pcl::PointXYZRGB>
                    > source (new faat_pcl::rec_3d_framework::ModelOnlySource<pcl::PointXYZRGBNormal, pcl::PointXYZRGB>);
            source->setPath (model_path);
            source->setLoadViews (false);
            source->setLoadIntoMemory(false);
            std::string test = "irrelevant";
            source->generate (test);

            pcl::visualization::PCLVisualizer::Ptr vis (new pcl::visualization::PCLVisualizer("vis1"));
            std::vector<int> viewportNr = visualization_framework (vis, nrFiles, SUBWINDOWS_PER_VIEW);
            vis->setWindowName ("Hypotheses generation and verification for single views");

            int s_id = 0;
            for (std::vector<Vertex>::iterator it_vrtx = vertices_v.begin (); it_vrtx != vertices_v.end (); ++it_vrtx, ++s_id)
            {
                //-------Visualize Scene Cloud--------------------
                pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> handler_rgb (grph[*it_vrtx].pScenePCl);
                std::stringstream cloud_name;
                cloud_name << "scene_cloud_" << grph[*it_vrtx].scene_filename;
                vis->addPointCloud<pcl::PointXYZRGB> (grph[*it_vrtx].pScenePCl, handler_rgb, cloud_name.str (), viewportNr[boost::get (vertex_index, grph, *it_vrtx) * SUBWINDOWS_PER_VIEW]);


                //ATTENTION: Fix this, do not loaad the whole cloud
                for (size_t hypVec_id = 0; hypVec_id < grph[*it_vrtx].hypothesis_single_all.size (); hypVec_id++)
                {
                    std::vector < std::string > strs_2;
                    boost::split (strs_2, grph[*it_vrtx].hypothesis_single_all[hypVec_id].model_id_, boost::is_any_of ("/\\"));
                    ModelTPtr model;
                    bool found = source->getModelById (strs_2[strs_2.size () - 1], model);

                    PointInTPtr pHypothesisAlignedPCl (new pcl::PointCloud<pcl::PointXYZRGB>);

                    if (found)
                    {
                        pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr pHypothesisPCl (new pcl::PointCloud<pcl::PointXYZRGB>);
                        pHypothesisPCl = model->getAssembled(0.005f);
                        pcl::transformPointCloud (*pHypothesisPCl, *pHypothesisAlignedPCl, grph[*it_vrtx].hypothesis_single_all[hypVec_id].transform_);
                    }
                    else
                    {
                        PointInTPtr pHypothesisPCl (new pcl::PointCloud<pcl::PointXYZRGB>);
                        pcl::io::loadPCDFile (grph[*it_vrtx].hypothesis_single_all[hypVec_id].model_id_, *pHypothesisPCl);
                        pcl::transformPointCloud (*pHypothesisPCl, *pHypothesisAlignedPCl, grph[*it_vrtx].hypothesis_single_all[hypVec_id].transform_);
                    }

                    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> handler_rgb_verified (pHypothesisAlignedPCl);
                    std::stringstream name;
                    name << "Hypothesis_model_single_unverified_" << grph[*it_vrtx].hypothesis_single_all[hypVec_id].model_id_ << "__forScene_" << grph[*it_vrtx].scene_filename
                    << "__origin_" << grph[*it_vrtx].hypothesis_single_all[hypVec_id].origin_ << "__with_vector_id_" << hypVec_id;
                    vis->addPointCloud<pcl::PointXYZRGB> (pHypothesisAlignedPCl, handler_rgb_verified, name.str (),
                                      viewportNr[boost::get (vertex_index, grph, *it_vrtx) * SUBWINDOWS_PER_VIEW + 1]);
                }

                for (size_t hypVec_id = 0; hypVec_id < grph[*it_vrtx].hypothesis.size (); hypVec_id++)
                {

                    std::vector < std::string > strs_2;
                    boost::split (strs_2, grph[*it_vrtx].hypothesis[hypVec_id].model_id_, boost::is_any_of ("/\\"));
                    ModelTPtr model;
                    bool found = source->getModelById (strs_2[strs_2.size () - 1], model);

                    PointInTPtr pHypothesisAlignedPCl (new pcl::PointCloud<pcl::PointXYZRGB>);
                    if (found)
                    {
                        pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr pHypothesisPCl (new pcl::PointCloud<pcl::PointXYZRGB>);
                        pHypothesisPCl = model->getAssembled(0.005f);
                        pcl::transformPointCloud (*pHypothesisPCl, *pHypothesisAlignedPCl, grph[*it_vrtx].hypothesis[hypVec_id].transform_);
                    }
                    else
                    {
                        PointInTPtr pHypothesisPCl (new pcl::PointCloud<pcl::PointXYZRGB>);
                        pcl::io::loadPCDFile (grph[*it_vrtx].hypothesis[hypVec_id].model_id_, *pHypothesisPCl);
                        pcl::transformPointCloud (*pHypothesisPCl, *pHypothesisAlignedPCl, grph[*it_vrtx].hypothesis[hypVec_id].transform_);
                    }

                    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> handler_rgb_verified (pHypothesisAlignedPCl);
                    std::stringstream name;
                    name << "Hypothesis_model_" << grph[*it_vrtx].hypothesis[hypVec_id].model_id_ << "__forScene_" << grph[*it_vrtx].scene_filename
                         << "__origin_" << grph[*it_vrtx].hypothesis[hypVec_id].origin_ << "__with_vector_id_" << hypVec_id;
                    vis->addPointCloud<pcl::PointXYZRGB> (pHypothesisAlignedPCl, handler_rgb_verified, name.str (),
                                                          viewportNr[boost::get (vertex_index, grph, *it_vrtx) * SUBWINDOWS_PER_VIEW + 2]);
                }

                for(size_t kk=0; kk < grph[*it_vrtx].verified_planes_.size(); kk++)
                {
                    std::stringstream pname;
                    pname << "plane_" << s_id << "_" << kk;

                    pcl::visualization::PointCloudColorHandlerRandom<PointT> scene_handler(grph[*it_vrtx].verified_planes_[kk].plane_cloud_);
                    vis->addPointCloud<PointT> (grph[*it_vrtx].verified_planes_[kk].plane_cloud_, scene_handler, pname.str(), viewportNr[boost::get (vertex_index, grph, *it_vrtx) * SUBWINDOWS_PER_VIEW + 2]);

                    pname << "chull";
                    vis->addPolygonMesh (*grph[*it_vrtx].verified_planes_[kk].convex_hull_, pname.str(), viewportNr[boost::get (vertex_index, grph, *it_vrtx) * SUBWINDOWS_PER_VIEW + 2]);
                }
            }
            //vis->resetCamera();
            //vis->setFullScreen(true);
            vis->setBackgroundColor(1,1,1);
            vis->spin ();
            vis->getInteractorStyle()->saveScreenshot("singleview.png");
        }
    }
    else
    {
        vertices_v = my_node_reader (hypothesis_file, grph);
        std::cout << "finished reading graph:" << boost::num_vertices(grph) << std::endl;
        single_view_hypothesis_exist = true;
    }

    for (std::vector<Vertex>::iterator it_vrtxA = vertices_v.begin (); it_vrtxA != vertices_v.end (); ++it_vrtxA)
    {
        Vertex vrtx_final = boost::add_vertex (grph_final);
        //--- estimate surface normals of the scene --------
        if(single_view_hypothesis_exist)
        {
            pcl::NormalEstimationOMP<PointT, pcl::Normal> ne;
            ne.setRadiusSearch(0.02f);
            ne.setInputCloud (grph[*it_vrtxA].pScenePCl);
            ne.compute (*(grph[*it_vrtxA].normals_));
        }

        //---copy-vertex-and-its-information-to-graph_final----------------------------
        grph_final[vrtx_final].pScenePCl = grph[*it_vrtxA].pScenePCl;
        //grph_final[vrtx_final].pScenePCl_f = grph[*it_vrtxA].pScenePCl_f;
        //grph_final[vrtx_final].pSceneNormal = grph[*it_vrtxA].pSceneNormal;
        grph_final[vrtx_final].normals_ = grph[*it_vrtxA].normals_;
        //grph_final[vrtx_final].pScenePCl_f_ds = grph[*it_vrtx].pScenePCl_f_ds;
        grph_final[vrtx_final].scene_filename = grph[*it_vrtxA].scene_filename;
        grph_final[vrtx_final].hypothesis = grph[*it_vrtxA].hypothesis;
        grph_final[vrtx_final].hypothesis_single_unverified = grph[*it_vrtxA].hypothesis_single_unverified;
        grph_final[vrtx_final].hypothesis_single_all = grph[*it_vrtxA].hypothesis_single_all;
    }

    if(!only_scene_to_scene)
        createEdgesFromHypothesisMatch (vertices_v, grph, edges);

    int lastindex = hypothesis_file.find_last_of(".");
    std::string hypothesis_raw_filename = hypothesis_file.substr(0, lastindex);

    std::string grph_output_filename = hypothesis_raw_filename + std::string("_single_view_results.dot");
    outputgraph (grph, hypothesis_file.c_str());
    outputgraph (grph, grph_output_filename.c_str ());

    //----------create-edges-between-views-by-SIFT-----------------------------------
    for (std::vector<Vertex>::iterator it_vrtxA = vertices_v.begin (); it_vrtxA != vertices_v.end (); ++it_vrtxA)
    {
        calcFeatures (*it_vrtxA, grph, use_table_plane);
        std::cout << "keypoints:" << grph[*it_vrtxA].pKeypoints->points.size() << std::endl;
        grph_final[*it_vrtxA].pKeypoints = grph[*it_vrtxA].pKeypoints;
        grph_final[*it_vrtxA].keypoints_indices_ = grph[*it_vrtxA].keypoints_indices_;
        grph_final[*it_vrtxA].sift_keypoints_scales = grph[*it_vrtxA].sift_keypoints_scales;

        if(!scene_to_scene)
            continue;

        flann::Matrix<float> flann_data;
        flann::Index<DistT> *flann_index;
        multiview::convertToFLANN<pcl::Histogram<128> > (grph[*it_vrtxA].pSignatures, flann_data);
        flann_index = new flann::Index<DistT> (flann_data, flann::KDTreeIndexParams (4));
        flann_index->buildIndex ();

        for (std::vector<Vertex>::iterator it_vrtxB = vertices_v.begin (); it_vrtxB != it_vrtxA; ++it_vrtxB)
        {
            Eigen::Matrix4f transformation;

            std::vector<Edge> edge;
            estimateViewTransformationBySIFT (*it_vrtxB, *it_vrtxA, grph, flann_index, transformation, edge, use_gc_s2s, max_node_distance_);

            for(size_t kk=0; kk < edge.size(); kk++)
            {
                edges.push_back (edge[kk]);
            }
        }

        delete flann_index;
    }

    calcEdgeWeight (edges, grph, max_node_distance_, z_dist, max_overlap_bf);
    std::vector<Edge> edges_final;
    calcMST (edges, grph, edges_final);


   outputgraph (grph, grph_output_filename.c_str ());

    //---copy-edge-information-from-the-edge-vector-to-the-final-graph-------------------------------------------------------------
    //#pragma omp parallel for
    for (size_t edgeVec_id = 0; edgeVec_id < edges_final.size (); edgeVec_id++)
    {
        Vertex vrtx_src, vrtx_trgt;
        vrtx_src = source (edges_final[edgeVec_id], grph);
        vrtx_trgt = target (edges_final[edgeVec_id], grph);

        Edge e_cpy;
        bool b;
        tie (e_cpy, b) = add_edge (vrtx_src, vrtx_trgt, grph_final);

        grph_final[e_cpy].transformation = grph[edges_final[edgeVec_id]].transformation;
        grph_final[e_cpy].edge_weight = grph[edges_final[edgeVec_id]].edge_weight;
        grph_final[e_cpy].model_name = grph[edges_final[edgeVec_id]].model_name;
        grph_final[e_cpy].source_id = grph[edges_final[edgeVec_id]].source_id;
        grph_final[e_cpy].target_id = grph[edges_final[edgeVec_id]].target_id;
    }

    grph_output_filename = hypothesis_raw_filename + std::string("_mst_without_backprojection.dot");
    outputgraph (grph_final, grph_output_filename.c_str ());

    //------Extend-hypotheses-from-other-view(s)------------------------------------------
    //extendHypothesis (grph_final); //is this necessary with GO3D?

    if (go_3d)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr big_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
        createBigPointCloud (grph_final, big_cloud);

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

        for (vp = vertices (grph_final); vp.first != vp.second; ++vp.first, ++idx)
        {

            pcl::PointCloud<pcl::PointXYZRGB>::Ptr trans_cloud (new pcl::PointCloud<pcl::PointXYZRGB>(*grph_final[*vp.first].pScenePCl));
            pcl::PointCloud<pcl::Normal>::Ptr normal_cloud (new pcl::PointCloud<pcl::Normal>(*grph_final[*vp.first].normals_));

            normal_clouds.push_back(grph_final[*vp.first].normals_);

            std::vector<bool> kept_map;
            kept_map.resize(trans_cloud->points.size(), false);

            std::vector<int> kept_indices;
            {
                bool depth_edges = true;

                faat_pcl::utils::noise_models::NguyenNoiseModel<pcl::PointXYZRGB> nm;
                nm.setInputCloud(trans_cloud);
                nm.setInputNormals(normal_cloud);
                nm.setLateralSigma(lateral_sigma);
                nm.setMaxAngle(max_angle);
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

            if(use_table_plane)
            {
                Eigen::Vector4f table_plane;
                myRecognizer::computeTablePlane (grph_final[*vp.first].pScenePCl, table_plane);
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
            faat_pcl::utils::miscellaneous::transformNormals(normal_cloud, normal_cloud_trans, grph_final[*vp.first].absolute_pose);

            if(mv_keypoints == 0)
                //using SIFT keypoints
            {
                //compute indices to original cloud (to address normals) that are not farther away that 1.3
                std::vector<int> sift_indices = grph_final[*vp.first].keypoints_indices_.indices;
                std::vector<int> original_indices;
                std::vector<float> keypoint_scales = grph_final[*vp.first].sift_keypoints_scales;

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

                pcl::transformPointCloud(*trans_cloud2, *trans_cloud, grph_final[*vp.first].absolute_pose);
                pcl::copyPointCloud(*normal_cloud_trans, original_indices, *normal_cloud);
            }
            else if(mv_keypoints == 1)
            {
                //using RGB edges
                std::vector<int> edge_indices;
                registration_utils::getRGBEdges<pcl::PointXYZRGB>(grph_final[*vp.first].pScenePCl, edge_indices, 175, 225, max_keypoint_dist_mv_);
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
                pcl::transformPointCloud(*trans_cloud2, *trans_cloud, grph_final[*vp.first].absolute_pose);
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
                pcl::transformPointCloud(*trans_cloud2, *trans_cloud, grph_final[*vp.first].absolute_pose);
            }

            pcl::PointCloud<pcl::PointXYZRGB>::Ptr trans_cloud2 (new pcl::PointCloud<pcl::PointXYZRGB>(*trans_cloud));

            original_clouds.push_back(grph_final[*vp.first].pScenePCl);
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

//        if(mv_icp_)
//        {
//            //refine registered scene clouds simulatenously and adapt transforms
//            std::vector < std::vector<bool> > A;
//            A.resize (object_clouds.size ());
//            for (size_t i = 0; i < object_clouds.size (); i++)
//                A[i].resize (object_clouds.size (), true);

//            faat_pcl::registration_utils::computeOverlapMatrix<pcl::PointXYZRGB>(object_clouds, A, 0.02, false, min_overlap_mv);

//            for (size_t i = 0; i < object_clouds.size (); i++)
//            {
//                for (size_t j = 0; j < object_clouds.size (); j++)
//                    std::cout << (int)A[i][j] << " ";
//                std::cout << std::endl;
//            }

//            faat_pcl::registration::MVNonLinearICP<PointT> icp_nl (dt_size);
//            icp_nl.setInlierThreshold (inlier_threshold);
//            icp_nl.setMaxCorrespondenceDistance (max_corresp_dist);
//            icp_nl.setClouds (object_clouds);

//            if(use_normals)
//            {
//                icp_nl.setInputNormals(normals_clouds);
//                icp_nl.setMinDot(0.9f);
//            }

//            icp_nl.setVisIntermediate (false);
//            icp_nl.setSparseSolver (true);
//            icp_nl.setMaxIterations(mv_iterations);
//            icp_nl.setAdjacencyMatrix (A);

//            if(mv_weights.size() == object_clouds.size())
//                icp_nl.setPointsWeight(mv_weights);

//            icp_nl.compute ();

//            icp_nl.getTransformation (transformations);

//            int kk=0;
//            for (vp = vertices (grph_final); vp.first != vp.second; ++vp.first, kk++)
//            {
//                grph_final[*vp.first].absolute_pose = transformations[kk] * grph_final[*vp.first].absolute_pose;
//            }
//        }

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
        source->setPath (model_path);
        source->setLoadViews (false);
        source->setLoadIntoMemory(false);
        std::string test = "irrelevant";
        source->generate (test);
        source->createVoxelGridAndDistanceTransform (go3d_and_icp_resolution_);

        std::vector<int> hyp_index_to_vp;
        int kk=0;
        for (vp = vertices (grph_final); vp.first != vp.second; ++vp.first, kk++)
        {
            std::cout << *vp.first << " " << kk << std::endl;
            //transforms_to_global.push_back (transformations[kk] * grph_final[*vp.first].absolute_pose);
            transforms_to_global.push_back (grph_final[*vp.first].absolute_pose);

            for (std::vector<Hypothesis>::iterator it_hyp = grph_final[*vp.first].hypothesis.begin (); it_hyp != grph_final[*vp.first].hypothesis.end (); ++it_hyp)
            {
                if(it_hyp->extended_)
                    continue;

                std::vector < std::string > strs_2;
                boost::split (strs_2, it_hyp->model_id_, boost::is_any_of ("/\\"));
                ModelTPtr model;
                source->getModelById (strs_2[strs_2.size () - 1], model);

                //Eigen::Matrix4f trans = transformations[kk] * grph_final[*vp.first].absolute_pose * it_hyp->transform_;
                Eigen::Matrix4f trans = grph_final[*vp.first].absolute_pose * it_hyp->transform_;
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
                std::cout << "use_unverified_single_view_hypotheses is true " << grph_final[*vp.first].hypothesis_single_unverified.size() <<  std::endl;
                for (std::vector<Hypothesis>::iterator it_hyp = grph_final[*vp.first].hypothesis_single_unverified.begin ();
                     it_hyp != grph_final[*vp.first].hypothesis_single_unverified.end (); ++it_hyp)
                {
                    if(it_hyp->extended_)
                        continue;

                    std::vector < std::string > strs_2;
                    boost::split (strs_2, ids[kk], boost::is_any_of ("/\\"));
                    ModelTPtr model;
                    bool found = source->getModelById (strs_2[strs_2.size () - 1], model);

                    //Eigen::Matrix4f trans = transformations[kk] * grph_final[*vp.first].absolute_pose * it_hyp->transform_;
                    Eigen::Matrix4f trans = grph_final[*vp.first].absolute_pose * it_hyp->transform_;
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
                        icp.setMaximumIterations(icp_iter);
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
                        reg.setMaximumIterations (icp_iter);
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
            std::cout << "go3d_outlier_regularizer:" << go3d_outlier_regularizer << std::endl;
            std::cout << "go3d_clutter_regularizer:" << go3d_clutter_regularizer << std::endl;
            std::cout << "go3d_use_supervoxels:" << go3d_use_supervoxels << std::endl;
            std::cout << "go3d_color_sigma:" << go3d_color_sigma << std::endl;

            faat_pcl::GO3D<PointT, PointT> go;
            go.setResolution (go3d_and_icp_resolution_);
            go.setAbsolutePoses (transforms_to_global);
            go.setSmoothSegParameters(0.1, 0.04f, 0.01f, 100);
            go.setUseSuperVoxels(go3d_use_supervoxels);
            go.setOcclusionsClouds (occlusion_clouds);
            go.setZBufferSelfOcclusionResolution (250);
            go.setInlierThreshold (go3d_inlier_threshold_);
            go.setRadiusClutter (go3d_clutter_radius_);
            go.setDetectClutter (go3d_detect_clutter); //Attention, detect clutter turned off!
            go.setRegularizer (go3d_outlier_regularizer);
            go.setClutterRegularizer (go3d_clutter_regularizer);
            go.setHypPenalty (0.05f);
            go.setIgnoreColor (false);
            go.setColorSigma (go3d_color_sigma);
            go.setOptimizerType (opt_type);
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

            if(visualize_output)
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

                if(go3d_add_planes)
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
                    for (vp = vertices (grph_final); vp.first != vp.second; ++vp.first, k++)
                    {
                        //hypotheses_poses_in_global_frame[i] transforms from object coordinates to global coordinate system
                        //transforms_to_global aligns a single frame to the global coordinate system
                        //transformation would then be a transformation transforming first the object to global coordinate system
                        //concatenated with the inverse of transforms_to_global[k]
                        Eigen::Matrix4f t = transforms_to_global[k].inverse() * hypotheses_poses_in_global_frame[i];
                        std::string origin = "3d go";
                        Hypothesis hyp(ids[i], t, origin, true, true);
                        grph_final[*vp.first].hypothesis.push_back(hyp);
                    }
                }
            }

            if(output_dir_3d_results.compare("") != 0)
            {
                bf::path out_dir_path = output_dir_3d_results;
                if(!bf::exists(out_dir_path))
                {
                    bf::create_directory(out_dir_path);
                }

                for(size_t i=0; i < occlusion_clouds.size(); i++)
                {
                    std::stringstream pose_path;
                    pose_path << output_dir_3d_results << "/transformation_" << setw(5) << setfill('0') << i << ".txt";
                    faat_pcl::utils::writeMatrixToFile(pose_path.str(), transforms_to_global[i]);

                    std::stringstream cloud_path;
                    cloud_path << output_dir_3d_results << "/cloud_" << setw(5) << setfill('0') << i << ".pcd";
                    pcl::io::savePCDFileBinary(cloud_path.str(), *occlusion_clouds[i]);

                    {
                        {
                            std::stringstream cloud_path;
                            cloud_path << output_dir_3d_results << "/original_clouds/";
                            bf::path out_dir_path = cloud_path.str();
                            if(!bf::exists(out_dir_path))
                            {
                                bf::create_directory(out_dir_path);
                            }
                        }

                        std::stringstream cloud_path;
                        cloud_path << output_dir_3d_results << "/original_clouds/cloud_" << setw(5) << setfill('0') << i << ".pcd";
                        pcl::io::savePCDFileBinary(cloud_path.str(), *original_clouds[i]);
                    }
                }

                std::stringstream results_path;
                results_path << output_dir_3d_results << "/results_3d.txt";

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
    /*else
    {

        //------Extend-hypotheses-from-other-view(s)------------------------------------------
        extendHypothesis (grph_final); //is this necessary with GO3D?

        //Refine hypotheses that where extended
        float icp_resolution_ = 0.005f;
        float icp_max_correspondence_distance_ = 0.02f;

        if (icp_iter > 0)
        {

            boost::shared_ptr < faat_pcl::rec_3d_framework::ModelOnlySource<pcl::PointXYZRGBNormal, pcl::PointXYZRGB>
                    > source (new faat_pcl::rec_3d_framework::ModelOnlySource<pcl::PointXYZRGBNormal, pcl::PointXYZRGB>);
            source->setPath (model_path);
            source->setLoadViews (false);
            source->setLoadIntoMemory(false);
            std::string test = "irrelevant";
            source->generate (test);
            source->createVoxelGridAndDistanceTransform (icp_resolution_);

            std::pair<vertex_iter, vertex_iter> vp;
            for (vp = vertices (grph_final); vp.first != vp.second; ++vp.first)
            {
                std::cout << "Hypotheses in this frame after extension:" << grph_final[*vp.first].hypothesis.size () << std::endl;

    #pragma omp parallel for num_threads(8)
                for (size_t kk = 0; kk < grph_final[*vp.first].hypothesis.size (); kk++)
                {

                    if (!grph_final[*vp.first].hypothesis[kk].extended_)
                        continue;

                    std::vector < std::string > strs_2;
                    boost::split (strs_2, grph_final[*vp.first].hypothesis[kk].model_id_, boost::is_any_of ("/\\"));
                    ModelTPtr model;
                    bool found = source->getModelById (strs_2[strs_2.size () - 1], model);
                    //std::cout << grph_final[*vp.first].hypothesis[kk].model_id << std::endl;
                    if (found)
                    {
                        //std::cout << model->id_ << std::endl;
                        boost::shared_ptr < distance_field::PropagationDistanceField<pcl::PointXYZRGB> > dt;
                        model->getVGDT (dt);

                        faat_pcl::rec_3d_framework::VoxelBasedCorrespondenceEstimation<pcl::PointXYZRGB, pcl::PointXYZRGB>::Ptr
                                est (
                                    new faat_pcl::rec_3d_framework::VoxelBasedCorrespondenceEstimation<
                                    pcl::PointXYZRGB,
                                    pcl::PointXYZRGB> ());

                        pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZRGB>::Ptr
                                rej (
                                    new pcl::registration::CorrespondenceRejectorSampleConsensus<
                                    pcl::PointXYZRGB> ());

                        Eigen::Matrix4f scene_to_model_trans = grph_final[*vp.first].hypothesis[kk].transform_.inverse ();

                        pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud;
                        dt->getInputCloud (cloud);

                        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_voxelized_icp_transformed (new pcl::PointCloud<pcl::PointXYZRGB> ());
                        pcl::transformPointCloud (*grph_final[*vp.first].pScenePCl, *cloud_voxelized_icp_transformed, scene_to_model_trans);

                        est->setVoxelRepresentationTarget (dt);
                        est->setInputSource (cloud_voxelized_icp_transformed);
                        est->setInputTarget (cloud);
                        est->setMaxCorrespondenceDistance (icp_max_correspondence_distance_);
                        est->setMaxColorDistance (-1, -1);

                        rej->setInputTarget (cloud);
                        rej->setMaximumIterations (1000);
                        rej->setInlierThreshold (0.005f);
                        rej->setInputSource (cloud_voxelized_icp_transformed);

                        pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> reg;
                        reg.setCorrespondenceEstimation (est);
                        reg.addCorrespondenceRejector (rej);
                        reg.setInputTarget (cloud); //model
                        reg.setInputSource (cloud_voxelized_icp_transformed); //scene
                        reg.setMaximumIterations (icp_iter);
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
                        grph_final[*vp.first].hypothesis[kk].transform_ = icp_trans;
                    }
                }
            }
        }

        //---Verify-extended-hypotheses-and-visualize------------------------
        std::pair<vertex_iter, vertex_iter> vp;
        for (vp = vertices (grph_final); vp.first != vp.second; ++vp.first)
        {

            std::vector<pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr> aligned_models;
            std::vector < std::string > ids;

            for (std::vector<Hypothesis>::iterator it_hyp = grph_final[*vp.first].hypothesis.begin (); it_hyp != grph_final[*vp.first].hypothesis.end (); ++it_hyp)
            {
                int vector_id = it_hyp - grph_final[*vp.first].hypothesis.begin ();
                PointInTPtr pModelPCl (new pcl::PointCloud<pcl::PointXYZRGB>);
                PointInTPtr pModelPCl2 (new pcl::PointCloud<pcl::PointXYZRGB>);
                pcl::io::loadPCDFile (it_hyp->model_id_, *(pModelPCl));

                pcl::transformPointCloud (*pModelPCl, *pModelPCl, it_hyp->transform_);

                pcl::VoxelGrid<pcl::PointXYZRGB> sor;
                float leaf = 0.005f;
                sor.setLeafSize (leaf, leaf, leaf);
                sor.setInputCloud (pModelPCl);
                sor.filter (*pModelPCl2);

                aligned_models.push_back (pModelPCl2);
                ids.push_back (it_hyp->model_id_);
            }
            std::cout << "View " << boost::get (vertex_index, grph_final, *vp.first) << " has " << grph_final[*vp.first].hypothesis.size ()
                      << " hypothesis. " << std::endl;

            boost::shared_ptr<faat_pcl::GlobalHypothesesVerification_1<PointT, PointT> > go (new faat_pcl::GlobalHypothesesVerification_1<PointT, PointT>);
            go->setUseConflictGraph (false);
            go->setIgnoreColor (false);
            go->setZBufferSelfOcclusionResolution (250);
            go->setOcclusionThreshold (0.0075f);
            go->setSelfOcclusionsReasoning (false);
            go->setResolution (0.005f);
            go->setInlierThreshold (0.01f);
            go->setRadiusClutter (0.035f);
            go->setRegularizer (1.f);
            go->setClutterRegularizer (3.f);
            go->setHypPenalty (0.f);
            go->setDetectClutter (true);
            go->setColorSigma (30.f);
            go->setOptimizerType (opt_type);
            go->setOcclusionCloud (grph_final[*vp.first].pScenePCl);

            boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > xyz_points_andy (new pcl::PointCloud<pcl::PointXYZRGB>);
            filterPCl (grph_final[*vp.first].pScenePCl, xyz_points_andy);

            go->setSceneCloud (xyz_points_andy);
            go->addModels (aligned_models, true);
            go->setObjectIds (ids);
            go->verify ();

            std::vector<bool> mask_hv;
            go->getMask (mask_hv);
            for (std::vector<bool>::iterator it_mask = mask_hv.begin (); it_mask != mask_hv.end (); ++it_mask)
            {
                int id_temp = it_mask - mask_hv.begin ();
                std::cout << id_temp << "is" << static_cast<int> (*it_mask) << std::endl;
                //std::cout << static_cast<int> (mask_hv[j]) << std::endl;
                if (!*it_mask)
                    continue;

                grph_final[*vp.first].hypothesis[id_temp].verified_ = true;
            }
        }
    }*/
    grph_output_filename = hypothesis_raw_filename + std::string("_mst_with_backprojection.dot");
    outputgraph (grph_final, grph_output_filename.c_str ());

    if(gt_or_ouput_dir.compare("") != 0)
    {
        bf::path or_path = gt_or_ouput_dir;
        if(!bf::exists(or_path))
        {
            bf::create_directory(or_path);
        }

        for (std::vector<Vertex>::iterator it_vrtx = vertices_v.begin (); it_vrtx != vertices_v.end (); ++it_vrtx)
        {
            std::stringstream fn_str;
            fn_str << gt_or_ouput_dir << "/";
            std::map<std::string, int> id_count;
            std::map<std::string, int>::iterator id_c_it;
            for (size_t hypVec_id = 0; hypVec_id < grph_final[*it_vrtx].hypothesis.size (); hypVec_id++)
            {

                if(!grph_final[*it_vrtx].hypothesis[hypVec_id].verified_)
                    continue;

                //std::cout << grph_final[*it_vrtx].scene_filename << std::endl;
                std::string scene (grph_final[*it_vrtx].scene_filename);
                boost::replace_all (scene, input_cloud_dir, "");
                boost::replace_all (scene, ".pcd", "");

                std::string model_id_replaced (grph_final[*it_vrtx].hypothesis[hypVec_id].model_id_);
                boost::replace_all (model_id_replaced, model_path, "");
                boost::replace_all (model_id_replaced, ".pcd", "");
                id_c_it = id_count.find(model_id_replaced);
                if(id_c_it == id_count.end())
                {
                    id_count[model_id_replaced] = 0;
                    id_c_it = id_count.find(model_id_replaced);
                }
                else
                {
                    id_c_it->second++;
                }

                //std::cout << model_id_replaced << " " << id_c_it->second << std::endl;
                std::stringstream pose_file;
                pose_file << fn_str.str() << scene << "_" << model_id_replaced << "_" << id_c_it->second << ".txt";
                std::cout << pose_file.str() << std::endl;
                faat_pcl::utils::writeMatrixToFile(pose_file.str(), grph_final[*it_vrtx].hypothesis[hypVec_id].transform_);
                //grph_final[*it_vrtx].hypothesis[hypVec_id].transform_
            }
        }
    }

    if(visualize_output_single_view)
    {
        pcl::visualization::PCLVisualizer::Ptr vis (new pcl::visualization::PCLVisualizer("vis2"));
        std::vector<int> viewportNr = visualization_framework (vis, vertices_v.size (), SUBWINDOWS_PER_VIEW_HT_FROM_FILE);
        vis->setWindowName ("Hypotheses transformation and verification for multiple views");
        for (std::vector<Vertex>::iterator it_vrtx = vertices_v.begin (); it_vrtx != vertices_v.end (); ++it_vrtx)
        {
            std::stringstream cloud_name_tmp;
            cloud_name_tmp << "scene_cloud_" << grph_final[*it_vrtx].scene_filename;
            pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> handler_rgb (grph_final[*it_vrtx].pScenePCl);
            vis->addPointCloud<pcl::PointXYZRGB> (grph_final[*it_vrtx].pScenePCl, handler_rgb, cloud_name_tmp.str (),
                                                  viewportNr[boost::get (vertex_index, grph_final, *it_vrtx) * SUBWINDOWS_PER_VIEW_HT_FROM_FILE + 0]);

            for (size_t hypVec_id = 0; hypVec_id < grph_final[*it_vrtx].hypothesis.size (); hypVec_id++)
            {

                PointInTPtr pHypothesisPCl (new pcl::PointCloud<pcl::PointXYZRGB>);
                PointInTPtr pHypothesisPCl_vx (new pcl::PointCloud<pcl::PointXYZRGB>);
                PointInTPtr pHypothesisAlignedPCl (new pcl::PointCloud<pcl::PointXYZRGB>);
                pcl::io::loadPCDFile (grph_final[*it_vrtx].hypothesis[hypVec_id].model_id_, *pHypothesisPCl);

                pcl::VoxelGrid<pcl::PointXYZRGB> sor;
                float leaf = 0.0025f;
                sor.setLeafSize (leaf, leaf, leaf);
                sor.setInputCloud (pHypothesisPCl);
                sor.filter (*pHypothesisPCl_vx);

                pcl::transformPointCloud (*pHypothesisPCl_vx, *pHypothesisAlignedPCl, grph_final[*it_vrtx].hypothesis[hypVec_id].transform_);
                pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> handler_rgb_verified (pHypothesisAlignedPCl);
                std::stringstream basename;
                basename << "Hypothesis_model_" << grph_final[*it_vrtx].hypothesis[hypVec_id].model_id_ << "__forScene_" << grph_final[*it_vrtx].scene_filename
                         << "__origin_" << grph_final[*it_vrtx].hypothesis[hypVec_id].origin_ << "__with_vector_id_" << hypVec_id;

                if(grph_final[*it_vrtx].hypothesis[hypVec_id].origin_.compare(grph_final[*it_vrtx].scene_filename) == 0)	//--show-hypotheses-from-single-view
                {
                    std::stringstream name;
                    name << "Single_View_" << basename.str();
                    vis->addPointCloud<pcl::PointXYZRGB> (pHypothesisAlignedPCl, handler_rgb_verified, name.str (),
                                                          viewportNr[boost::get (vertex_index, grph_final, *it_vrtx) * SUBWINDOWS_PER_VIEW_HT_FROM_FILE + 1]);
                }

                /*std::stringstream name;
      name << "After_Hyp_Extension_" << basename.str();
      vis->addPointCloud<pcl::PointXYZRGB> (pHypothesisAlignedPCl, handler_rgb_verified, name.str (),
                          viewportNr[boost::get (vertex_index, grph_final, *it_vrtx) * SUBWINDOWS_PER_VIEW_HT_FROM_FILE + 2]);*/


                if(grph_final[*it_vrtx].hypothesis[hypVec_id].verified_)	//--show-verified-hypotheses
                {
                    //std::cout << grph_final[*it_vrtx].hypothesis[hypVec_id].transform_ << std::endl;
                    std::stringstream name;
                    name << "After_Hyp_Extension_and_Verification_" << basename.str();
                    vis->addPointCloud<pcl::PointXYZRGB> (pHypothesisAlignedPCl, handler_rgb_verified, name.str (),
                                                          viewportNr[boost::get (vertex_index, grph_final, *it_vrtx) * SUBWINDOWS_PER_VIEW_HT_FROM_FILE + 3]);
                }
            }
        }
        vis->setBackgroundColor(1,1,1);
        vis->resetCamera();
        //vis->setFullScreen(true);
        vis->spin ();
        vis->getInteractorStyle()->saveScreenshot("multiview.png");
    }
}

