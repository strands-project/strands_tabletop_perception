#ifndef MYGRAPHCLASSES_H
#define MYGRAPHCLASSES_H

#include <vector>
#include <iostream>
#include <string>
#include <sstream>
#include <boost/graph/adjacency_list.hpp>
#include <boost/filesystem.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/graphviz.hpp>
#include <boost/graph/prim_minimum_spanning_tree.hpp>
#include <v4r/ORFramework/sift_local_estimator.h>
#include <v4r/ORFramework/faat_3d_rec_framework_defines.h>
#include <v4r/ORRecognition/ghv.h>
#include <opencv2/core/core.hpp>
#include <opencv2/nonfree/features2d.hpp>
//#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/features/normal_3d.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/pfh.h>
#include <pcl/features/vfh.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/correspondence_types.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/search/impl/flann_search.hpp>
#include <pcl/search/kdtree.h>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointInT;
typedef PointInT::ConstPtr ConstPointInTPtr;
typedef boost::shared_ptr< PointInT > PointInTPtr;
typedef pcl::PointXYZRGB PointT;
typedef pcl::Histogram<128> FeatureT;
typedef flann::L1<float> DistT;
using namespace boost;

//class View;

class Hypothesis
{
public:
  Hypothesis(std::string model_id, Eigen::Matrix4f transform, std::string origin, bool extended = false, bool verified = false);
  std::string model_id_, origin_;
  Eigen::Matrix4f transform_;
  bool extended_;
  bool verified_;
};

class View
{
public:
  View();
  //View(const View &view);
  int foo;
  boost::shared_ptr< pcl::PointCloud<pcl::PointXYZRGB> > pScenePCl;
  boost::shared_ptr< pcl::PointCloud<pcl::PointXYZRGBNormal> > pSceneNormal;
  pcl::PointCloud<pcl::Normal>::Ptr normals_;
  boost::shared_ptr< pcl::PointCloud<pcl::PointXYZRGB> > pScenePCl_f; //no table plane
  boost::shared_ptr< pcl::PointCloud<pcl::PointXYZRGB> > pScenePCl_f_ds;
  boost::shared_ptr< pcl::PointCloud<FeatureT > > pSignatures;
  boost::shared_ptr< pcl::PointIndices > pIndices_above_plane;
  boost::shared_ptr< pcl::PointCloud<pcl::PointXYZRGB> > pKeypoints;
  std::vector<float> sift_keypoints_scales;
  pcl::PointIndices keypoints_indices_;
  std::string scene_filename;
  std::vector<std::string> model_ids;
  std::vector<double> modelToViewCost;
  std::vector<Hypothesis> hypothesis;
  std::vector<Hypothesis> hypothesis_single_unverified;
  std::vector<Hypothesis> hypothesis_single_all;
  Eigen::Matrix4f absolute_pose;	
  std::vector<faat_pcl::PlaneModel<pcl::PointXYZRGB> > verified_planes_;
};

class myEdge
{
public:
  Eigen::Matrix4f transformation;
  double edge_weight;
  std::string model_name;
  int source_id, target_id;
  std::vector <cv::DMatch> matches;
};


typedef adjacency_list < vecS, vecS, undirectedS, property<vertex_distance_t, int>, property<edge_weight_t, double> > GraphMST;
typedef boost::graph_traits < GraphMST >::vertex_descriptor VertexMST;
typedef boost::graph_traits < GraphMST >::edge_descriptor EdgeMST;

//--"copy"-of-graph-to-save-custom-information------prim_minimum_spanning_tree----cannot(?)-handle-internal-bundled-properties---
typedef adjacency_list < vecS, vecS, undirectedS, View, myEdge > Graph;
typedef boost::graph_traits < Graph >::vertex_descriptor Vertex;
typedef boost::graph_traits < Graph >::edge_descriptor Edge;
typedef boost::graph_traits<Graph>::vertex_iterator vertex_iter;
typedef property_map<Graph, vertex_index_t>::type IndexMap;
std::vector<Vertex> my_node_reader(std::string filename, Graph &g);

namespace multiview
{
  void
  nearestKSearch (flann::Index<flann::L1<float> > * index,
                    float * descr, int descr_size,
                    int k,flann::Matrix<int> &indices,flann::Matrix<float> &distances);

  template <typename Type>
 void
  convertToFLANN (typename pcl::PointCloud<Type>::Ptr & cloud, flann::Matrix<float> &data);
}

void createBigPointCloud(Graph & grph_final,
                            pcl::PointCloud<pcl::PointXYZRGB>::Ptr & big_cloud);

bool calcFeatures(Vertex &src, Graph &grph, bool use_table_plane=true);
void estimateViewTransformationBySIFT(const Vertex &src, const Vertex &trgt, Graph &grph, flann::Index<DistT > *flann_index, Eigen::Matrix4f &transformation, std::vector<Edge> & edge, bool use_gc=false, int max_node_distance=-1);
void selectLowestWeightEdgesFromParallelEdges(const std::vector<Edge> &parallel_edges, const Graph &grph, std::vector<Edge> &single_edges);
void extendHypothesis(Graph &grph);
void calcMST(const std::vector<Edge> &edges, const Graph &grph, std::vector<Edge> &edges_final);
void createEdgesFromHypothesisMatch(const std::vector<Vertex> &vertices_v, Graph &grph, std::vector<Edge> &edges);
void calcEdgeWeight(std::vector<Edge> &edges, Graph &grph, int max_distance=-1, float z_dist=3.f, float max_overlap=0.75f);
/*double calcRegistrationCost(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr pInputNormalPCl, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr pSceneNormalPCl,pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB>::Ptr pOctree, int K=1, double beta = 0.2);
double calcRegistrationCost(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr pInputNormalPCl, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr pSceneNormalPCl, pcl::search::OrganizedNeighbor<pcl::PointXYZRGB>::Ptr pOrganizedNeighbor, int K=1, double beta=0.2);
double calcRegistrationCost(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr pInputNormalPCl,
                                pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr pSceneNormalPCl,
                                std::vector<int> & unused, double beta=0.2);*/

#endif
