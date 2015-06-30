#include <thread>

#include <pcl/impl/instantiate.hpp>
#include <pcl/conversions.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_visualizer/remote_pcl_visualizer.h>

#include <pcl_visualizer_srv_definitions/SetWindowName.h>
#include <pcl_visualizer_srv_definitions/RemoveAllPointClouds.h>
#include <pcl_visualizer_srv_definitions/RemoveAllShapes.h>
#include <pcl_visualizer_srv_definitions/SetBackgroundColor.h>
#include <pcl_visualizer_srv_definitions/CreateViewport.h>
#include <pcl_visualizer_srv_definitions/AddText.h>
#include <pcl_visualizer_srv_definitions/AddPointCloud.h>

RemotePCLVisualizer::RemotePCLVisualizer(const std::string& title)
: nh_("~")
{
  setWindowName(title);
}

void RemotePCLVisualizer::setWindowName(const std::string& name)
{
  pcl_visualizer_srv_definitions::SetWindowName srv;
  srv.request.window_name = name;
  callService("set_window_name", srv);
}

void RemotePCLVisualizer::removeAllPointClouds(int viewport)
{
  pcl_visualizer_srv_definitions::RemoveAllPointClouds srv;
  srv.request.viewport = viewport;
  callService("remove_all_point_clouds", srv);
}

void RemotePCLVisualizer::removeAllShapes(int viewport)
{
  pcl_visualizer_srv_definitions::RemoveAllShapes srv;
  srv.request.viewport = viewport;
  callService("remove_all_shapes", srv);
}

void RemotePCLVisualizer::createViewPort(double xmin, double ymin, double xmax, double ymax, int& viewport)
{
  pcl_visualizer_srv_definitions::CreateViewport srv;
  srv.request.xmin = xmin;
  srv.request.ymin = ymin;
  srv.request.xmax = xmax;
  srv.request.ymax = ymax;
  callService("create_viewport", srv);
  viewport = srv.response.viewport;
}

void RemotePCLVisualizer::spin()
{
  while (true)
    std::this_thread::sleep_for(std::chrono::seconds(1));
}

void RemotePCLVisualizer::spinOnce(int time, bool force_redraw)
{
  std::this_thread::sleep_for(std::chrono::milliseconds(time));
}

bool RemotePCLVisualizer::addText(const std::string& text, int xpos, int ypos, const std::string& id, int viewport)
{
  pcl_visualizer_srv_definitions::AddText srv;
  srv.request.text = text;
  srv.request.xpos = xpos;
  srv.request.ypos = ypos;
  srv.request.id = id;
  srv.request.viewport = viewport;
  callService("add_text", srv);
  return srv.response.result;
}

void RemotePCLVisualizer::setBackgroundColor(double r, double g, double b, int viewport)
{
  pcl_visualizer_srv_definitions::SetBackgroundColor srv;
  srv.request.r = r;
  srv.request.g = g;
  srv.request.b = b;
  srv.request.viewport = viewport;
  callService("set_background_color", srv);
}

template <typename PointT>
bool RemotePCLVisualizer::addPointCloud(const typename pcl::PointCloud<PointT>::ConstPtr& cloud,
                                        const pcl::visualization::PointCloudColorHandler<PointT>& color_handler,
                                        const pcl::visualization::PointCloudGeometryHandler<PointT>& geometry_handler,
                                        const std::string& id, int viewport)
{
  // with color handler
  pcl_visualizer_srv_definitions::AddPointCloud srv;
  if (color_handler.getName() == "PointCloudColorHandlerCustom")
  {
    vtkSmartPointer<vtkDataArray> colors;
    color_handler.getColor(colors);
    if (colors->GetNumberOfTuples())
    {
      double* c = colors->GetTuple(0);
      srv.request.color_handler.r = c[0];
      srv.request.color_handler.g = c[1];
      srv.request.color_handler.b = c[2];
    }
    srv.request.color_handler.type = pcl_visualizer_srv_definitions::ColorHandler::CUSTOM;
  }
  else if (color_handler.getName() == "PointCloudColorHandlerRGBField")
  {
    srv.request.color_handler.type = pcl_visualizer_srv_definitions::ColorHandler::RGB;
  }
  else if (color_handler.getName() == "PointCloudColorHandlerRGBAField")
  {
    srv.request.color_handler.type = pcl_visualizer_srv_definitions::ColorHandler::RGBA;
  }
  else
  {
    throw std::runtime_error(color_handler.getName() + " is not supported by addPointCloud");
  }
  srv.request.id = id;
  srv.request.viewport = viewport;
  pcl::PCLPointCloud2 pcl_cloud;
  pcl::toPCLPointCloud2(*cloud, pcl_cloud);
  pcl_conversions::moveFromPCL(pcl_cloud, srv.request.cloud);
  callService("add_point_cloud", srv);
  return srv.response.result;
}

template <typename PointT>
bool RemotePCLVisualizer::addPointCloud(const typename pcl::PointCloud<PointT>::ConstPtr& cloud,
                                        const pcl::visualization::PointCloudGeometryHandler<PointT>& geometry_handler,
                                        const std::string& id, int viewport)
{
  // without color handler
  pcl_visualizer_srv_definitions::AddPointCloud srv;
  srv.request.id = id;
  srv.request.viewport = viewport;
  pcl::PCLPointCloud2 pcl_cloud;
  pcl::toPCLPointCloud2(*cloud, pcl_cloud);
  pcl_conversions::moveFromPCL(pcl_cloud, srv.request.cloud);
  callService("add_point_cloud", srv);
  return srv.response.result;
}

template <typename PointT>
bool RemotePCLVisualizer::addPointCloud(const typename pcl::PointCloud<PointT>::ConstPtr& cloud,
                                        const std::string& id, int viewport)
{
  pcl::visualization::PointCloudGeometryHandlerXYZ<PointT> geometry_handler(cloud);
  addPointCloud(cloud, geometry_handler, id, viewport);
}

std::vector<int> RemotePCLVisualizer::createFramework(size_t number_of_views,
                                                      size_t number_of_subwindows_per_view,
                                                      const std::vector<std::string>& title_subwindows)
{
  std::vector<int> viewport_nr(number_of_views * number_of_subwindows_per_view, 0);

  for (size_t i = 0; i < number_of_views; i++)
  {
    for (size_t j = 0; j < number_of_subwindows_per_view; j++)
    {
      createViewPort(float (i) / number_of_views, float (j) / number_of_subwindows_per_view, (float (i) + 1.0) / number_of_views,
                     float (j + 1) / number_of_subwindows_per_view, viewport_nr[number_of_subwindows_per_view * i + j]);

      setBackgroundColor(float (j * ((i % 2) / 10.0 + 1)) / number_of_subwindows_per_view,
                         float (j * ((i % 2) / 10.0 + 1)) / number_of_subwindows_per_view,
                         float (j * ((i % 2) / 10.0 + 1)) / number_of_subwindows_per_view, viewport_nr[number_of_subwindows_per_view * i + j]);

      removeAllShapes(viewport_nr[i * number_of_subwindows_per_view + j]);
      std::stringstream window_id;
      window_id << "(" << i << ", " << j << ") ";
      if (title_subwindows.size() > j)
      {
        window_id << title_subwindows[j];
      }
      addText(window_id.str(), 10, 10, window_id.str(), viewport_nr[i * number_of_subwindows_per_view + j]);
    }
  }
  return viewport_nr;
}

template<typename T>
void RemotePCLVisualizer::callService(const std::string& name, T& service)
{
  std::string uri = "pcl_visualizer/" + name;
  if (!ros::service::call<T>(uri, service))
    throw std::runtime_error("service call to " + uri + " failed");
}

#define PCL_INSTANTIATE_addPointCloud(T) template bool RemotePCLVisualizer::addPointCloud<T>(const pcl::PointCloud<T>::ConstPtr&, const std::string&, int);
#define PCL_INSTANTIATE_addPointCloudG(T) template bool RemotePCLVisualizer::addPointCloud<T>(const pcl::PointCloud<T>::ConstPtr&, const pcl::visualization::PointCloudGeometryHandler<T>&, const std::string&, int);
#define PCL_INSTANTIATE_addPointCloudCG(T) template bool RemotePCLVisualizer::addPointCloud<T>(const pcl::PointCloud<T>::ConstPtr&, const pcl::visualization::PointCloudColorHandler<T>&, const pcl::visualization::PointCloudGeometryHandler<T>&, const std::string&, int);

PCL_INSTANTIATE(addPointCloud, PCL_XYZ_POINT_TYPES)
PCL_INSTANTIATE(addPointCloudG, PCL_XYZ_POINT_TYPES)
PCL_INSTANTIATE(addPointCloudCG, PCL_XYZ_POINT_TYPES)

