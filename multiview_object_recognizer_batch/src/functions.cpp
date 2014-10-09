/*
 * functions.cpp
 *
 *  Created on: Apr 12, 2013
 *      Author: Thomas Fäulhammer
 *
 *      Definition of functions used in multiview_gt project
 *
 */

#include "functions.h"

void
getFilesInDirect (bf::path & dir, std::string & rel_path_so_far, std::vector<std::string> & relative_paths, std::string & ext)
{
  bf::directory_iterator end_itr;
  for (bf::directory_iterator itr (dir); itr != end_itr; ++itr)
  {
    //check if its a directory, then get models in it
    if (bf::is_directory (*itr))
    {
#if BOOST_FILESYSTEM_VERSION == 3
      std::string so_far = rel_path_so_far + (itr->path ().filename ()).string() + "/";
#else
      std::string so_far = rel_path_so_far + (itr->path ()).filename () + "/";
#endif

      bf::path curr_path = itr->path ();
      getFilesInDirect (curr_path, so_far, relative_paths, ext);
    }
    else
    {
      //check that it is a ply file and then add, otherwise ignore..
      std::vector < std::string > strs;
#if BOOST_FILESYSTEM_VERSION == 3
      std::string file = (itr->path ().filename ()).string();
#else
      std::string file = (itr->path ()).filename ();
#endif

      boost::split (strs, file, boost::is_any_of ("."));
      std::string extension = strs[strs.size () - 1];

      if (extension.compare (ext) == 0)
      {
#if BOOST_FILESYSTEM_VERSION == 3
        std::string path = rel_path_so_far + (itr->path ().filename ()).string();
#else
        std::string path = rel_path_so_far + (itr->path ()).filename ();
#endif

        relative_paths.push_back (path);
      }
    }
  }
}

void
help (int argc, char **argv)
{
  std::cout << "Usage: " << argv[0] << "[-models_dir_sift md -training_dir td -model_path mp -input_cloud_dir ic | -hypothesis ht]" << std::endl;
  std::cout << "md... Directory containing models\n" << "td... Directory containing training data\n" << "mp... Directory containing models\n"
      << "ic... Directory containing input clouds from different views\n" << "ht... File containing hypothesis for the different views" << std::endl;
}

std::vector<int>
visualization_framework (pcl::visualization::PCLVisualizer::Ptr vis, int number_of_views, int number_of_subwindows_per_view)
{
  std::vector<int> viewportNr (number_of_views * number_of_subwindows_per_view, 0);

  for (size_t i = 0; i < number_of_views; i++)
  {
    for (size_t j = 0; j < number_of_subwindows_per_view; j++)
    {
      vis->createViewPort (float (i) / number_of_views, float (j) / number_of_subwindows_per_view, (float (i) + 1.0) / number_of_views,
                           float (j + 1) / number_of_subwindows_per_view, viewportNr[number_of_subwindows_per_view * i + j]);

      vis->setBackgroundColor (float (j * ((i % 2) / 10.0 + 1)) / number_of_subwindows_per_view,
                               float (j * ((i % 2) / 10.0 + 1)) / number_of_subwindows_per_view,
                               float (j * ((i % 2) / 10.0 + 1)) / number_of_subwindows_per_view, viewportNr[number_of_subwindows_per_view * i + j]);

      std::stringstream window_id;
      window_id << "(" << i << ", " << j << ")";
      vis->addText (window_id.str (), 10, 10, window_id.str (), viewportNr[i * number_of_subwindows_per_view + j]);
    }
  }
  return viewportNr;
}

void
filterPCl (boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZRGB> > pInputCloud, boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > pOutputCloud,
           pcl::PointIndices::Ptr pIndices_above_plane, float dist)
{
  
  Eigen::Vector4f table_plane;
  
  myRecognizer::computeTablePlane (pInputCloud, table_plane);

  pcl::PointIndices test;

  pcl::PassThrough<pcl::PointXYZRGB> pass;
  pass.setFilterLimits (0.f, dist);
  pass.setFilterFieldName ("z");
  pass.setInputCloud (pInputCloud);
  pass.setKeepOrganized (true);
  pass.filter (*pOutputCloud);
;
  //#pragma omp parallel for
  for (size_t kk = 0; kk < pOutputCloud->points.size (); kk++)
  {
    
    Eigen::Vector3f xyz_p = pOutputCloud->points[kk].getVector3fMap ();

    if (!pcl_isfinite ( xyz_p[0] ) || !pcl_isfinite ( xyz_p[1] ) || !pcl_isfinite ( xyz_p[2] ))
      continue;

    float val = xyz_p[0] * table_plane[0] + xyz_p[1] * table_plane[1] + xyz_p[2] * table_plane[2] + table_plane[3];

    if (val <= 0.01)
    {
      pOutputCloud->points[kk].x = std::numeric_limits<float>::quiet_NaN ();
      pOutputCloud->points[kk].y = std::numeric_limits<float>::quiet_NaN ();
      pOutputCloud->points[kk].z = std::numeric_limits<float>::quiet_NaN ();
    }
    else
    {
      pIndices_above_plane->indices.push_back (kk);
    }
  }

  //         //----downsample point cloud-----
  //         pcl::VoxelGrid < pcl::PointXYZRGB > sor;
  //         sor.setInputCloud ( grph[*it_vrtx].pScenePCl_f );
  //         sor.setLeafSize ( 0.01f, 0.01f, 0.01f );
  //         sor.filter ( * ( grph[*it_vrtx].pScenePCl_f_ds ) );
}

void
filterPCl (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr pInputCloud, boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > pOutputCloud,
           float dist)
{
  Eigen::Vector4f table_plane;
  myRecognizer::computeTablePlane (pInputCloud, table_plane);

  pcl::PassThrough<pcl::PointXYZRGB> pass;
  pass.setFilterLimits (0.f, dist);
  pass.setFilterFieldName ("z");
  pass.setInputCloud (pInputCloud);
  pass.setKeepOrganized (true);
  pass.filter (*pOutputCloud);

#pragma omp parallel for
  for (size_t kk = 0; kk < pOutputCloud->points.size (); kk++)
  {
    Eigen::Vector3f xyz_p = pOutputCloud->points[kk].getVector3fMap ();

    if (!pcl_isfinite ( xyz_p[0] ) || !pcl_isfinite ( xyz_p[1] ) || !pcl_isfinite ( xyz_p[2] ))
      continue;

    float val = xyz_p[0] * table_plane[0] + xyz_p[1] * table_plane[1] + xyz_p[2] * table_plane[2] + table_plane[3];

    if (val <= 0.01)
    {
      pOutputCloud->points[kk].x = std::numeric_limits<float>::quiet_NaN ();
      pOutputCloud->points[kk].y = std::numeric_limits<float>::quiet_NaN ();
      pOutputCloud->points[kk].z = std::numeric_limits<float>::quiet_NaN ();
    }

  }
}

void
ConvertPCLCloud2Image (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &pcl_cloud, cv::Mat_<cv::Vec3b> &image)
{
  unsigned pcWidth = pcl_cloud->width;
  unsigned pcHeight = pcl_cloud->height;
  unsigned position = 0;

  image = cv::Mat_<cv::Vec3b> (pcHeight, pcWidth);

  for (unsigned row = 0; row < pcHeight; row++)
  {
    for (unsigned col = 0; col < pcWidth; col++)
    {
      cv::Vec3b &cvp = image.at<cv::Vec3b> (row, col);
      position = row * pcWidth + col;
      const pcl::PointXYZRGB &pt = pcl_cloud->points[position];

      cvp[0] = pt.b;
      cvp[1] = pt.g;
      cvp[2] = pt.r;
    }
  }
}

void transformNormals(pcl::PointCloud<pcl::Normal>::Ptr & normals_cloud,
                      pcl::PointCloud<pcl::Normal>::Ptr & normals_aligned,
                      Eigen::Matrix4f & transform)
{
    normals_aligned.reset (new pcl::PointCloud<pcl::Normal>);
    normals_aligned->points.resize (normals_cloud->points.size ());
    normals_aligned->width = normals_cloud->width;
    normals_aligned->height = normals_cloud->height;
    for (size_t k = 0; k < normals_cloud->points.size (); k++)
    {
        Eigen::Vector3f nt (normals_cloud->points[k].normal_x, normals_cloud->points[k].normal_y, normals_cloud->points[k].normal_z);
        normals_aligned->points[k].normal_x = static_cast<float> (transform (0, 0) * nt[0] + transform (0, 1) * nt[1]
                                                                  + transform (0, 2) * nt[2]);
        normals_aligned->points[k].normal_y = static_cast<float> (transform (1, 0) * nt[0] + transform (1, 1) * nt[1]
                                                                  + transform (1, 2) * nt[2]);
        normals_aligned->points[k].normal_z = static_cast<float> (transform (2, 0) * nt[0] + transform (2, 1) * nt[1]
                                                                  + transform (2, 2) * nt[2]);
    }
}
