#ifndef REMOTE_PCL_VISUALIZER_H
#define REMOTE_PCL_VISUALIZER_H

#include <string>

#include <ros/ros.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/visualization/point_cloud_geometry_handlers.h>

class RemotePCLVisualizer
{

  public:

    RemotePCLVisualizer(const std::string& title = "");

    virtual ~RemotePCLVisualizer() { };

    void setWindowName(const std::string& name);

    void removeAllPointClouds(int viewport = 0);

    void removeAllShapes(int viewport = 0);

    void createViewPort(double xmin, double ymin, double xmax, double ymax, int& viewport);

    void spin();

    void spinOnce(int time = 1, bool force_redraw = false);

    bool addText(const std::string& text, int xpos, int ypos, const std::string& id = "", int viewport = 0);

    void setBackgroundColor(double r, double g, double b, int viewport = 0);

    template <typename PointT>
    bool addPointCloud(const typename pcl::PointCloud<PointT>::ConstPtr& cloud,
                       const pcl::visualization::PointCloudColorHandler<PointT>& color_handler,
                       const pcl::visualization::PointCloudGeometryHandler<PointT>& geometry_handler,
                       const std::string& id = "cloud", int viewport = 0);

    template <typename PointT>
    bool addPointCloud(const typename pcl::PointCloud<PointT>::ConstPtr& cloud,
                       const pcl::visualization::PointCloudGeometryHandler<PointT>& geometry_handler,
                       const std::string& id = "cloud", int viewport = 0);

    template <typename PointT>
    bool addPointCloud(const typename pcl::PointCloud<PointT>::ConstPtr& cloud,
                       const pcl::visualization::PointCloudColorHandler<PointT>& color_handler,
                       const std::string& id = "cloud", int viewport = 0)
    {
      pcl::visualization::PointCloudGeometryHandlerXYZ<PointT> geometry_handler(cloud);
      addPointCloud(cloud, color_handler, geometry_handler, id, viewport);
    }

    template <typename PointT>
    bool addPointCloud(const typename pcl::PointCloud<PointT>::ConstPtr& cloud,
                       const std::string& id = "cloud", int viewport = 0);

    inline bool addPointCloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud,
                              const std::string &id = "cloud", int viewport = 0)
    {
      return addPointCloud<pcl::PointXYZ> (cloud, id, viewport);
    }

    inline bool addPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud,
                              const std::string &id = "cloud", int viewport = 0)
    {
      pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> color_handler(cloud);
      return addPointCloud<pcl::PointXYZRGB> (cloud, color_handler, id, viewport);
    }

    inline bool addPointCloud(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr& cloud,
                              const std::string &id = "cloud", int viewport = 0)
    {
      pcl::visualization::PointCloudColorHandlerRGBAField<pcl::PointXYZRGBA> color_handler(cloud);
      return addPointCloud<pcl::PointXYZRGBA> (cloud, color_handler, id, viewport);
    }

    std::vector<int> createFramework(size_t number_of_views,
                                     size_t number_of_subwindows_per_view,
                                     const std::vector<std::string>& title_subwindows = std::vector<std::string>());

  private:

    template<typename T>
    void callService(const std::string& uri, T& service);

    ros::NodeHandle nh_;

};

#endif /* REMOTE_PCL_VISUALIZER_H */

