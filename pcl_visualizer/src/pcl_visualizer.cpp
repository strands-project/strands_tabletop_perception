#include <pcl/PCLPointCloud2.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_geometry_handlers.h>
#include <pcl/visualization/point_cloud_color_handlers.h>

#include <ros/ros.h>
#include <ros/console.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_visualizer_srv_definitions/SetWindowName.h>
#include <pcl_visualizer_srv_definitions/RemoveAllPointClouds.h>
#include <pcl_visualizer_srv_definitions/RemoveAllShapes.h>
#include <pcl_visualizer_srv_definitions/SetBackgroundColor.h>
#include <pcl_visualizer_srv_definitions/CreateViewport.h>
#include <pcl_visualizer_srv_definitions/AddText.h>
#include <pcl_visualizer_srv_definitions/AddPointCloud.h>

class PCLVisualizerNode
{

  public:

    PCLVisualizerNode()
    : visualizer_("PCL Visualizer")
    , nh_("~")
    {
      server_set_window_name_ = nh_.advertiseService("set_window_name", &PCLVisualizerNode::setWindowNameCallback, this);
      server_remove_all_point_clouds_ = nh_.advertiseService("remove_all_point_clouds", &PCLVisualizerNode::removeAllPointCloudsCallback, this);
      server_remove_all_shapes_ = nh_.advertiseService("remove_all_shapes", &PCLVisualizerNode::removeAllShapesCallback, this);
      server_set_background_color_ = nh_.advertiseService("set_background_color", &PCLVisualizerNode::setBackgroundColorCallback, this);
      server_create_viewport_ = nh_.advertiseService("create_viewport", &PCLVisualizerNode::createViewportCallback, this);
      server_add_text_ = nh_.advertiseService("add_text", &PCLVisualizerNode::addTextCallback, this);
      server_add_point_cloud_ = nh_.advertiseService("add_point_cloud", &PCLVisualizerNode::addPointCloudCallback, this);
    }

    virtual ~PCLVisualizerNode()
    {
    }

    void run()
    {
      while (!ros::isShuttingDown())
      {
        visualizer_.spinOnce(10);
        ros::spinOnce();
      }
    }

    bool setWindowNameCallback(pcl_visualizer_srv_definitions::SetWindowName::Request& request,
                               pcl_visualizer_srv_definitions::SetWindowName::Response& response)
    {
      ROS_DEBUG("[pcl_visualizer/set_window_name] %s", request.window_name.c_str());
      visualizer_.setWindowName(request.window_name);
      return true;
    }

    bool removeAllPointCloudsCallback(pcl_visualizer_srv_definitions::RemoveAllPointClouds::Request& request,
                                      pcl_visualizer_srv_definitions::RemoveAllPointClouds::Response& response)
    {
      ROS_DEBUG("[pcl_visualizer/remove_all_point_clouds] viewport: %i", request.viewport);
      visualizer_.removeAllPointClouds(request.viewport);
      return true;
    }

    bool removeAllShapesCallback(pcl_visualizer_srv_definitions::RemoveAllShapes::Request& request,
                                 pcl_visualizer_srv_definitions::RemoveAllShapes::Response& response)
    {
      ROS_DEBUG("[pcl_visualizer/remove_all_shapes] viewport: %i", request.viewport);
      visualizer_.removeAllShapes(request.viewport);
      return true;
    }

    bool setBackgroundColorCallback(pcl_visualizer_srv_definitions::SetBackgroundColor::Request& request,
                                    pcl_visualizer_srv_definitions::SetBackgroundColor::Response& response)
    {
      ROS_DEBUG("[pcl_visualizer/set_background_color] rgb: %.2f %.2f %.2f, viewport: %i", request.r, request.g, request.b, request.viewport);
      visualizer_.setBackgroundColor(request.r, request.g, request.b, request.viewport);
      return true;
    }

    bool createViewportCallback(pcl_visualizer_srv_definitions::CreateViewport::Request& request,
                                pcl_visualizer_srv_definitions::CreateViewport::Response& response)
    {
      ROS_DEBUG("[pcl_visualizer/create_viewport] min: %.2f %.2f, max: %.2f %.2f", request.xmin, request.ymin, request.xmax, request.ymax);
      visualizer_.createViewPort(request.xmin, request.ymin, request.xmax, request.ymax, response.viewport);
      ROS_DEBUG("[pcl_visualizer/create_viewport] --> viewport: %i", response.viewport);
      return true;
    }

    bool addTextCallback(pcl_visualizer_srv_definitions::AddText::Request& request,
                         pcl_visualizer_srv_definitions::AddText::Response& response)
    {
      ROS_DEBUG("[pcl_visualizer/add_text] text: %s, pos: %i %i, id: %s, viewport: %i", request.text.c_str(), request.xpos, request.ypos, request.id.c_str(), request.viewport);
      response.result = visualizer_.addText(request.text, request.xpos, request.ypos, request.id, request.viewport);
      ROS_DEBUG("[pcl_visualizer/add_text] --> %s", response.result ? "true" : "false");
      return true;
    }

    bool addPointCloudCallback(pcl_visualizer_srv_definitions::AddPointCloud::Request& request,
                               pcl_visualizer_srv_definitions::AddPointCloud::Response& response)
    {
      typedef pcl::PCLPointCloud2 PointCloud;
      typedef pcl::visualization::PointCloudColorHandler<PointCloud> PointCloudColorHandler;
      typedef pcl::visualization::PointCloudGeometryHandlerXYZ<PointCloud> PointCloudGeometryHandlerXYZ;
      PointCloud::Ptr cloud(new PointCloud);
      pcl_conversions::moveToPCL(request.cloud, *cloud);
      PointCloudColorHandler::Ptr color_handler;
      PointCloudGeometryHandlerXYZ::Ptr geometry_handler(new PointCloudGeometryHandlerXYZ(cloud));
      if (request.color_handler.type == pcl_visualizer_srv_definitions::ColorHandler::CUSTOM)
      {
        color_handler.reset(new pcl::visualization::PointCloudColorHandlerCustom<PointCloud>(cloud, request.color_handler.r, request.color_handler.g, request.color_handler.b));
      }
      else if (request.color_handler.type == pcl_visualizer_srv_definitions::ColorHandler::RGB)
      {
        color_handler.reset(new pcl::visualization::PointCloudColorHandlerRGBField<PointCloud>(cloud));
      }
      else if (request.color_handler.type == pcl_visualizer_srv_definitions::ColorHandler::RGBA)
      {
        color_handler.reset(new pcl::visualization::PointCloudColorHandlerRGBField<PointCloud>(cloud));
      }
      else
      {
        // unsupported color handler
        return false;
      }
      ROS_DEBUG("[pcl_visualizer/add_point_cloud] color: %s, id: %s, viewport: %i", color_handler->getName().c_str(), request.id.c_str(), request.viewport);
      response.result = visualizer_.addPointCloud(cloud, geometry_handler, color_handler, Eigen::Vector4f(), Eigen::Quaternion<float>(), request.id, request.viewport);
      ROS_DEBUG("[pcl_visualizer/add_point_cloud] --> %s", response.result ? "true" : "false");
      return true;
    }

  private:

    pcl::visualization::PCLVisualizer visualizer_;

    ros::NodeHandle nh_;
    ros::ServiceServer server_set_window_name_;
    ros::ServiceServer server_remove_all_point_clouds_;
    ros::ServiceServer server_remove_all_shapes_;
    ros::ServiceServer server_set_background_color_;
    ros::ServiceServer server_create_viewport_;
    ros::ServiceServer server_add_text_;
    ros::ServiceServer server_add_point_cloud_;

};

int main(int argc, char** argv)
{
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
    ros::console::notifyLoggerLevelsChanged();

  ros::init(argc, argv, "pcl_visualizer");
  PCLVisualizerNode node;
  node.run();
  return 0;
}

