#include "my_filter.h"
#include <pcl/io/pcd_io.h>
#include <pcl/io/io.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <iostream>
#include <boost/filesystem.hpp>
#include <string>
 
#include <pcl/surface/convex_hull.h>

//#include <faat_pcl/utils/segmentation_utils.h>
//#include <faat_pcl/3d_rec_framework/defines/faat_3d_rec_framework_defines.h>
#include <pcl/apps/dominant_plane_segmentation.h>

struct IndexPoint
{
  int idx;
};

POINT_CLOUD_REGISTER_POINT_STRUCT (IndexPoint,
    (int, idx, idx)
)

namespace bf = boost::filesystem;

typedef pcl::PointXYZRGB PointT;

void segmentObjectFromTableTop(pcl::PointCloud<PointT>::Ptr pInput_cloud, pcl::PointIndices &indices_above_plane, float z_max = 1.5f)
{
    std::vector<pcl::PointCloud<PointT>::Ptr > clusters;
    std::vector<pcl::PointIndices> indices_above_plane_v;
    Eigen::Vector4f table_plane;
    //segmentation_utils::computeTablePlane<PointT>(pInput_cloud, table_plane, z_max);
    boost::shared_ptr< pcl::apps::DominantPlaneSegmentation<PointT>  > dps (new pcl::apps::DominantPlaneSegmentation<PointT>());
    dps->setInputCloud(pInput_cloud);
    dps->setMaxZBounds(z_max);
    //dps->compute_table_plane();
    dps->compute_fast(clusters);
    dps->getTableCoefficients(table_plane);
    dps->getIndicesClusters(indices_above_plane_v);

    for(size_t cluster_id = 0; cluster_id < indices_above_plane_v.size(); cluster_id++)
    {
        for(size_t kk=0; kk < indices_above_plane_v[cluster_id].indices.size(); kk++)
        {
            indices_above_plane.indices.push_back(indices_above_plane_v[cluster_id].indices[kk]);
        }
    }

    /*for (size_t k = 0; k < pInput_cloud->points.size (); k++)
    {
        Eigen::Vector3f xyz_p = pInput_cloud->points[k].getVector3fMap ();

        if (!pcl_isfinite (xyz_p[0]) || !pcl_isfinite (xyz_p[1]) || !pcl_isfinite (xyz_p[2]))
            continue;

        float val = xyz_p[0] * table_plane[0] + xyz_p[1] * table_plane[1] + xyz_p[2] * table_plane[2] + table_plane[3];

        if (val >= 0.01)
        {
            indices_above_plane.push_back (static_cast<int> (k));
        }
    }*/
}

void getFilenamesFromFilename(bf::path & dir, std::vector<std::string> & file_v)
{
    bf::directory_iterator end_itr;
    for (bf::directory_iterator itr (dir); itr != end_itr; ++itr)
    {
        std::string path;
#if BOOST_FILESYSTEM_VERSION == 3
          path = dir.string() + "/" +  (itr->path ().filename ()).string();
#else
          path = dir + "/" +  (itr->path ()).filename ();
#endif

        if (bf::is_directory (*itr))
        {
            bf::path path_bf = path;
            getFilenamesFromFilename(path_bf, file_v);
        }
        else
        {
            file_v.push_back(path);
        }
    }
}

void
getFoldersInDirectory (bf::path & dir, std::string & rel_path_so_far, std::vector<std::string> & relative_paths)
{
  bf::directory_iterator end_itr;
  for (bf::directory_iterator itr (dir); itr != end_itr; ++itr)
  {
    //check if its a directory, else ignore
    if (bf::is_directory (*itr))
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

void TableTopFilter::filterAndWriteToFile(const std::string path)
{
    std::string directory, filename;
    char sep = '/';
     #ifdef _WIN32
        sep = '\\';
     #endif

    size_t position = path.rfind(sep);
       if (position != std::string::npos)
       {
          directory = path.substr(0, position);
          filename = path.substr(position+1, path.length()-1);
       }

   std::stringstream path_oi;
   path_oi << directory << "/" << indices_prefix_ << filename ;

    if(bf::exists(path_oi.str()) && !force_refilter_)
    {
        std::cout << filename << " is already filtered and no re-filtering desired. " << std::endl;
        return;
    }

    if(filename.length() > indices_prefix_.length())
    {
        if(filename.compare(0, indices_prefix_.length(), indices_prefix_)==0 )
        {
            std::cout << filename << " is not a point cloud. " << std::endl;
            return;
        }
    }

    std::cout << "Filtering point cloud: " << filename << std::endl;

    boost::shared_ptr<pcl::PointCloud<PointT> > pCloud, pSegmentedCloud;
    pcl::PointIndices indices;
    pcl::PointCloud<IndexPoint> obj_indices_cloud;

    pCloud.reset(new pcl::PointCloud<PointT>());
    pSegmentedCloud.reset(new pcl::PointCloud<PointT>());
    pcl::io::loadPCDFile(path, *pCloud);

    segmentObjectFromTableTop(pCloud, indices, chop_z_max_);

    obj_indices_cloud.width = indices.indices.size();
    obj_indices_cloud.height = 1;
    obj_indices_cloud.points.resize(indices.indices.size());

    for(size_t kk=0; kk < indices.indices.size(); kk++)
    {
        obj_indices_cloud.points[kk].idx = indices.indices[kk];
    }

    if(obj_indices_cloud.points.size()==0)
    {
        std::cerr << "No points could be segmented above the table plane for file " <<
                     path << std::endl;
    }

    else
    {
        pcl::io::savePCDFileBinary(path_oi.str(), obj_indices_cloud);

        if(visualize_)
        {
            pcl::io::loadPCDFile (path_oi.str(), obj_indices_cloud);

            pcl::PointIndices indicesFinal;
            indicesFinal.indices.resize(obj_indices_cloud.points.size());
            for(size_t kk=0; kk < obj_indices_cloud.points.size(); kk++)
                  indicesFinal.indices[kk] = obj_indices_cloud.points[kk].idx;

            pcl::copyPointCloud(*pCloud, indicesFinal, *pSegmentedCloud);
            pcl::visualization::PCLVisualizer::Ptr vis;
            vis.reset(new pcl::visualization::PCLVisualizer("Filter result"));
            int v1(0), v2(0);
            vis->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
            vis->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
            pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb_handler (pCloud);
            vis->addPointCloud<PointT> (pCloud, rgb_handler, "input", v1);
            pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb_handler2 (pSegmentedCloud);
            vis->addPointCloud<PointT> (pSegmentedCloud, rgb_handler2, "filtered pcl", v2);
            vis->resetCameraViewpoint();
            vis->spin();
        }
    }
}

void TableTopFilter::filterAndWriteToFileRecursive(const std::string path)
{
    std::vector < std::string > folders;
    std::string start = "";
    bf::path input_dir_bf = path;

    getFoldersInDirectory (input_dir_bf, start, folders);
    std::cout << "There are " << folders.size() << " folders. " << std::endl;

    for (size_t i = 0; i < folders.size (); i++)
    {
        std::stringstream sub_dir;
        sub_dir << path << "/" << folders[i];
        filterAndWriteToFileRecursive(sub_dir.str());
    }

    bf::path sub_dir_bf = path;
    std::vector<std::string> file_v;
    getFilenamesFromFilename(sub_dir_bf, file_v);

    for(size_t i=0; i<file_v.size(); i++)
    {
        filterAndWriteToFile(file_v[i]);
    }
}

int main (int argc, char ** argv)
{
    TableTopFilter ttf;
    ttf.init(argc, argv);

    std::string input_dir = ttf.getInputDir();
    if(bf::is_directory(input_dir))
    {
        ttf.filterAndWriteToFileRecursive(input_dir);
    }
    else
    {
        std::cout << "Input directory: " << input_dir << " is not a directory." << std::endl;
    }
    return 0;
}
