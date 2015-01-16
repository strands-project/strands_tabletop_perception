#ifndef BOOST_GRAPH_EXTENSION_H
#define BOOST_GRAPH_EXTENSION_H
#include <boost/graph/adjacency_list.hpp>
#include <boost/filesystem.hpp>
#include <boost/graph/graph_traits.hpp>
//#include <boost/graph/prim_minimum_spanning_tree.hpp>
#include <pcl/PointIndices.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <v4r/ORFramework/source.h>
#include <v4r/ORFramework/recognizer.h>

typedef pcl::Histogram<128> FeatureT;


template<typename PointInT>
class Hypothesis
{
    typedef faat_pcl::rec_3d_framework::Model<PointInT> ModelT;
    typedef boost::shared_ptr<ModelT> ModelTPtr;

    static size_t sNum_hypotheses_;

public:
    ModelTPtr model_;
    std::string model_id_, origin_;
    Eigen::Matrix4f transform_;
    bool extended_;
    bool verified_;
    size_t id_;

    // deprecated interface------
    Hypothesis ( const std::string model_id, const Eigen::Matrix4f transform, const std::string origin = "", const bool extended = false, const bool verified = false, const size_t origin_id = 0)
    {
        model_id_ = model_id;
        transform_ = transform;
        origin_ = origin;
        extended_ = extended;
        verified_ = verified;

        if(extended && (origin_id < 0 || origin_id > sNum_hypotheses_))
        {
            std::cerr << "Hypothesis got extended but does not have a valid origin id." << std::endl;
        }

        if(origin_id)
        {
            id_ = origin_id;
        }
        else
        {
            id_ = ++sNum_hypotheses_;
        }
    }

    Hypothesis ( const ModelTPtr model, const Eigen::Matrix4f transform, const std::string origin = "", const bool extended = false, const bool verified = false, const size_t origin_id = 0)
    {
        model_ = model;
        model_id_ = model->id_;
        transform_ = transform;
        origin_ = origin;
        extended_ = extended;
        verified_ = verified;

        if(extended && (origin_id < 0 || origin_id > sNum_hypotheses_))
        {
            std::cerr << "Hypothesis got extended but does not have a valid origin id." << std::endl;
        }

        if(origin_id)
        {
            id_ = origin_id;
        }
        else
        {
            id_ = ++sNum_hypotheses_;
        }
    }
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
    boost::shared_ptr< pcl::PointCloud<pcl::Normal> > pSceneNormals_f;
    pcl::PointIndices filteredSceneIndices_;
    boost::shared_ptr< pcl::PointCloud<PointT> > pKeypointsMultipipe_;
    boost::shared_ptr< pcl::PointCloud<pcl::Normal> > pKeypointNormalsMultipipe_;
    std::map<std::string, faat_pcl::rec_3d_framework::ObjectHypothesis<PointT> > hypotheses_;
    boost::shared_ptr< pcl::PointCloud<FeatureT > > pSiftSignatures_;
    std::vector<float> sift_keypoints_scales_;
    pcl::PointIndices siftKeypointIndices_;
    std::vector<Hypothesis<PointT> > hypothesis_sv_;
    std::vector<Hypothesis<PointT> > hypothesis_mv_;
    Eigen::Matrix4f transform_to_world_co_system_;
    bool has_been_hopped_;
    double cumulative_weight_to_new_vrtx_;
    pcl::PointIndices keypointIndices_;
    std::vector<pcl::PointCloud<PointT>::Ptr> verified_planes_;

    //GO3D
    Eigen::Matrix4f absolute_pose_;
    std::vector<float> nguyens_noise_model_weights_;
    std::vector<int> nguyens_kept_indices_;
};

class myEdge
{
public:
    myEdge();
    Eigen::Matrix4f transformation;
    double edge_weight;
    std::string model_name;
    std::string source_id, target_id;
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
void outputgraph ( Graph &map, const char* filename );
void resetHopStatus(Graph &grph);
Vertex getFurthestVertex ( Graph &grph);
//void shallowCopyVertexIntoOtherGraph(const Vertex vrtx_src, const Graph grph_src, Vertex &vrtx_target, Graph &grph_target);
//void copyEdgeIntoOtherGraph(const Edge edge_src, const Graph grph_src, Edge &edge_target, Graph &grph_target);
//std::vector<Vertex> my_node_reader ( std::string filename, Graph &g )

template<typename PointInT> size_t Hypothesis<PointInT>::sNum_hypotheses_ = 0;

#endif
