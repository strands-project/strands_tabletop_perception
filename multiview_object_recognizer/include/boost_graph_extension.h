#ifndef BOOST_GRAPH_EXTENSION_H
#define BOOST_GRAPH_EXTENSION_H
#include <boost/graph/adjacency_list.hpp>
#include <boost/filesystem.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/graphviz.hpp>
//#include <boost/graph/prim_minimum_spanning_tree.hpp>
#include <pcl/PointIndices.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/core/core.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <pcl/visualization/pcl_visualizer.h>

#include <faat_pcl/3d_rec_framework/pipeline/recognizer.h>

typedef pcl::Histogram<128> FeatureT;


class Hypothesis
{
public:
    Hypothesis ( std::string model_id, Eigen::Matrix4f transform, std::string origin, bool extended = false, bool verified = false );
    std::string model_id_, origin_;
    Eigen::Matrix4f transform_;
    bool extended_;
    bool verified_;
};

class View
{
private:
    typedef pcl::PointXYZRGB PointT;

public:
    View();
    //View(const View &view);
    boost::shared_ptr< pcl::PointCloud<PointT> > pScenePCl;
    boost::shared_ptr< pcl::PointCloud<PointT> > pScenePCl_f;
    boost::shared_ptr< pcl::PointCloud<pcl::Normal> > pSceneNormals;
    boost::shared_ptr< pcl::PointCloud<pcl::Normal> > pSceneNormals_f_;
    boost::shared_ptr< pcl::PointCloud<pcl::Normal> > pKeypointNormals_;
    boost::shared_ptr< pcl::PointCloud<FeatureT > > pSignatures;
    boost::shared_ptr< pcl::PointIndices > pIndices_above_plane;
    boost::shared_ptr< pcl::PointCloud<PointT> > pKeypoints;
    boost::shared_ptr< pcl::PointCloud<PointT> > pKeypointsMultipipe_;
    std::map<std::string, faat_pcl::rec_3d_framework::ObjectHypothesis<PointT> > hypotheses_;
    std::vector<float> sift_keypoints_scales;
    pcl::PointIndices siftKeypointIndices_;
    std::string view_id_;
    //std::vector<std::string> model_ids_;
    std::vector<double> modelToViewCost;
    std::vector<Hypothesis> hypothesis;
    std::vector<Hypothesis> hypothesis_single_unverified;
    Eigen::Matrix4f absolute_pose;
    Eigen::Matrix4f transform_to_world_co_system_;
    bool has_been_hopped_;
    double cumulative_weight_to_new_vrtx_;
    size_t timestamp_nsec;
    pcl::PointIndices keypointIndices_;
};

class myEdge
{
public:
    myEdge();
    Eigen::Matrix4f transformation;
    double edge_weight;
    std::string model_name;
    std::string source_id, target_id;
    std::vector <cv::DMatch> matches;
    bool edge_weight_has_been_calculated_;
};


using namespace boost;

typedef adjacency_list < vecS, vecS, undirectedS, View, myEdge > Graph;
typedef graph_traits < Graph >::vertex_descriptor Vertex;
typedef graph_traits < Graph >::edge_descriptor Edge;
typedef graph_traits<Graph>::vertex_iterator vertex_iter;
typedef graph_traits<Graph>::edge_iterator edge_iter;
typedef property_map<Graph, vertex_index_t>::type IndexMap;


void visualizeGraph ( const Graph & grph, pcl::visualization::PCLVisualizer::Ptr vis);
void pruneGraph (Graph &grph, size_t num_remaining_vertices=2);
void outputgraph ( Graph& map, const char* filename );
Vertex getFurthestVertex ( Graph &grph);
void copyVertexIntoOtherGraph(const Vertex vrtx_src, const Graph grph_src, Vertex &vrtx_target, Graph &grph_target);
void copyEdgeIntoOtherGraph(const Edge edge_src, const Graph grph_src, Edge &edge_target, Graph &grph_target);
//std::vector<Vertex> my_node_reader ( std::string filename, Graph &g )
#endif
