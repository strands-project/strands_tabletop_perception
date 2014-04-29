#include "visualize_pcd_indices.h"
#include <math.h>

namespace bf = boost::filesystem;
typedef pcl::PointXYZRGB PointT;


std::string indices_prefix_ = "object_indices_";


struct IndexPoint
{
    int idx;
};

POINT_CLOUD_REGISTER_POINT_STRUCT (IndexPoint,
                                   (int, idx, idx)
                                   )


void
getFoldersInDirectory (const std::string dir, const std::string rel_path_so_far, std::vector<std::string> & relative_paths)
{
    bf::path dir_bf = dir;
    bf::directory_iterator end_itr;
    for (bf::directory_iterator itr (dir_bf); itr != end_itr; ++itr)
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

void getFilenamesFromFolder(const std::string dir, std::vector<std::string> & file_v)
{
    bf::path dir_bf = dir;
    bf::directory_iterator end_itr;
    for (bf::directory_iterator itr (dir_bf); itr != end_itr; ++itr)
    {
        std::string path;
#if BOOST_FILESYSTEM_VERSION == 3
        path = dir_bf.string() + "/" +  (itr->path ().filename ()).string();
#else
        path = dir + "/" +  (itr->path ()).filename ();
#endif

        if (bf::is_directory (*itr))
        {
            getFilenamesFromFolder(path, file_v);
        }
        else
        {
            file_v.push_back(path);
        }
    }
}

void getFileList(const std::string dir, std::vector<std::string> &fileList_v)
{
    std::vector<std::string> file_v;
    getFilenamesFromFolder(dir, file_v);

    for(size_t i=0; i<file_v.size(); i++)
    {
        std::string path = file_v[i];
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

        std::stringstream path_indices;
        path_indices << directory << "/" << indices_prefix_ << filename ;

        if(bf::exists(path_indices.str()))
        {
            fileList_v.push_back(path);
        }
    }
}


int
main (int argc, char ** argv)
{
    ros::init(argc, argv, "visualize_pcd_indices_node");
    ros::NodeHandle n("~");
    std::string dir;

    if(! n.getParam ( "dir", dir ))
    {
        std::cerr << "No input dir set. " << std::endl;
        return -1;
    }

    std::vector <std::string> fileList_v;
    getFileList(dir, fileList_v);

    size_t numFiles = fileList_v.size();
    std::cout << "Visualizing " << numFiles << " PCD files..." << std::endl;

    double sqrt_lf = sqrt(numFiles);
    int num_cols = std::floor(sqrt_lf);
    int num_rows;
    if(num_cols*num_cols == numFiles)
        num_rows = num_cols;
    else
        num_rows = num_cols+1;

    boost::shared_ptr<pcl::visualization::PCLVisualizer> vis;
    std::stringstream ss;
    ss << "Visualize indexed points of directory " << dir;
    vis.reset(new pcl::visualization::PCLVisualizer(ss.str()));
    std::vector<int> viewport_v;
    viewport_v.resize(numFiles);

    for(size_t i=0; i<fileList_v.size(); i++)
    {
        int current_row = std::floor(static_cast<double>(i) / num_cols);
        int current_col = i % num_cols;
std:cout << fileList_v[i] << std::endl;

        vis->createViewPort(
                    static_cast<double>(current_col)/num_cols,
                    static_cast<double>(current_row)/num_rows,
                    static_cast<double>(current_col+1)/num_cols,
                    static_cast<double>(current_row+1)/num_rows,
                    viewport_v[i]);

        boost::shared_ptr<pcl::PointCloud<PointT> > pCloud, pSegmentedCloud, pSegmentedCloudDemeaned;
        pCloud.reset(new pcl::PointCloud<PointT>());
        pSegmentedCloud.reset(new pcl::PointCloud<PointT>());
        pSegmentedCloudDemeaned.reset(new pcl::PointCloud<PointT>());
        pcl::io::loadPCDFile (fileList_v[i], *pCloud);

        std::string directory, filename;
        char sep = '/';
#ifdef _WIN32
        sep = '\\';
#endif

        size_t position = fileList_v[i].rfind(sep);
        if (position != std::string::npos)
        {
            directory = fileList_v[i].substr(0, position);
            filename = fileList_v[i].substr(position+1, fileList_v[i].length()-1);
        }

        std::stringstream path_indices;
        path_indices << directory << "/" << indices_prefix_ << filename ;

        pcl::PointCloud<IndexPoint> obj_indices_cloud;
        pcl::io::loadPCDFile (path_indices.str(), obj_indices_cloud);

        pcl::PointIndices indices;
        indices.indices.resize(obj_indices_cloud.points.size());
        for(size_t kk=0; kk < obj_indices_cloud.points.size(); kk++)
            indices.indices[kk] = obj_indices_cloud.points[kk].idx;

        pcl::copyPointCloud(*pCloud, indices, *pSegmentedCloud);
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*pSegmentedCloud, centroid);
        pcl::demeanPointCloud(*pSegmentedCloud, centroid, *pSegmentedCloudDemeaned);
        pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb_handler (pSegmentedCloudDemeaned);
        std::stringstream cloud_name;
        cloud_name << fileList_v[i].substr(dir.length(), fileList_v[i].length()-1);
        vis->addPointCloud<PointT> (pSegmentedCloudDemeaned, rgb_handler, cloud_name.str(), viewport_v[i]);
        vis->addText(cloud_name.str(), 10, 10, cloud_name.str(), viewport_v[i]);
    }
    vis->spin();
}
