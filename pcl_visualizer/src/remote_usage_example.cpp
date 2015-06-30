#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <ros/ros.h>

#include <pcl_visualizer/remote_pcl_visualizer.h>

template<typename PointT>
void generateXYZ(pcl::PointCloud<PointT>& cloud)
{
  cloud.width = 50;
  cloud.height = 1;
  cloud.is_dense = false;
  cloud.points.resize(cloud.width * cloud.height);
  for (size_t i = 0; i < cloud.points.size(); ++i)
  {
    cloud.points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
    cloud.points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
    cloud.points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
  }
}

template<typename PointT>
void generateRGB(pcl::PointCloud<PointT>& cloud)
{
  for (size_t i = 0; i < cloud.points.size(); ++i)
  {
    cloud.points[i].r = 255 * (rand() / (RAND_MAX + 1.0f));
    cloud.points[i].g = 255 * (rand() / (RAND_MAX + 1.0f));
    cloud.points[i].b = 255 * (rand() / (RAND_MAX + 1.0f));
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "remote_usage_example");
  RemotePCLVisualizer visualizer("foobar");
  std::vector<int> vp = visualizer.createFramework(2, 2);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr color_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red(xyz_cloud, 255, 20, 0);
  generateXYZ(*color_cloud);
  generateRGB(*color_cloud);
  generateXYZ(*xyz_cloud);
  visualizer.addPointCloud(color_cloud, "color-cloud", vp[0]);
  visualizer.addPointCloud(xyz_cloud, red, "xyz-cloud", vp[1]);

  ros::spin();
  return 0;
}
