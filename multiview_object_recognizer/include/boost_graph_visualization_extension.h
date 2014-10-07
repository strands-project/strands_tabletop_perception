#ifndef BOOST_GRAPH_VISUALIZATION_EXTENSION_H
#define BOOST_GRAPH_VISUALIZATION_EXTENSION_H

#include <vector>
#include <iostream>
#include <string>
#include <sstream>

#include <opencv2/core/core.hpp>
#include <opencv2/nonfree/features2d.hpp>

#include <pcl/common/transforms.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>

#include <faat_pcl/3d_rec_framework/pc_source/model_only_source.h>
#include <faat_pcl/utils/pcl_visualization_utils.h>
#include <faat_pcl/utils/pcl_opencv.h>

#include <pcl_conversions.h>
#include "boost_graph_extension.h"

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointInT;
typedef PointInT::ConstPtr ConstPointInTPtr;
typedef boost::shared_ptr< PointInT > PointInTPtr;

typedef faat_pcl::rec_3d_framework::Model<PointT> ModelT;
typedef boost::shared_ptr<ModelT> ModelTPtr;
typedef typename pcl::PointCloud<PointT>::ConstPtr ConstPointInTPtr;

class BoostGraphVisualizer
{
private:
    pcl::visualization::PCLVisualizer::Ptr edge_vis_;
    pcl::visualization::PCLVisualizer::Ptr keypoints_vis_;
    int v1, v2;

public:
    BoostGraphVisualizer(){

    }

    void visualizeEdge (const Edge &edge, const Graph &grph);

    void visualizeGraph ( const Graph & grph, pcl::visualization::PCLVisualizer::Ptr &vis);

    void visualizeWorkflow ( const Vertex &vrtx, const Graph &grph, boost::shared_ptr< pcl::PointCloud<PointT> > pAccumulatedKeypoints);

    void createImage(const Vertex &src, const Graph &grph, std::string outputfile);
};

#endif // BOOST_GRAPH_VISUALIZATION_EXTENSION_H
