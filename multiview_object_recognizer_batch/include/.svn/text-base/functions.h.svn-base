/*
 * functions.h
 *
 *  Created on: Apr 12, 2013
 *      Author: Thomas Fäulhammer
 *
 *      Declaration of functions used in multiview_gt project
 */

#ifndef FUNCTIONS_H_
#define FUNCTIONS_H_

#include <faat_pcl/3d_rec_framework/pc_source/partial_pcd_source.h>
#include <boost/filesystem.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/core/core.hpp>
#include <pcl/filters/passthrough.h>
#include <pcl/octree/octree.h>
#include <pcl/point_cloud.h>
#include "myRecognizer.h"
#include <omp.h>
#include <flann/flann.h>

namespace bf = boost::filesystem;

void getFilesInDirect (bf::path & dir, std::string & rel_path_so_far, std::vector<std::string> & relative_paths, std::string & ext);
void help(int argc, char **argv);
std::vector<int> visualization_framework(pcl::visualization::PCLVisualizer::Ptr vis, int number_of_views, int number_of_subwindows_per_view);

void filterPCl(boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZRGB> > pInputCloud, boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > pOutputCloud, pcl::PointIndices::Ptr indices_above_plane, float dist=1.5f);
void filterPCl(boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZRGB> > pInputCloud, boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > pOutputCloud, float dist=1.5f);
void ConvertPCLCloud2Image(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &pcl_cloud, cv::Mat_<cv::Vec3b> &image);
void transformNormals(pcl::PointCloud<pcl::Normal>::Ptr & normals_cloud,
                      pcl::PointCloud<pcl::Normal>::Ptr & normals_aligned,
                      Eigen::Matrix4f & transform);


#endif /* FUNCTIONS_H_ */
