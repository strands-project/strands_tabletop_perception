#include <iostream>

#include "ros/ros.h"

#include "strands_perception_msgs/Table.h"

#include "visualization_msgs/Marker.h"

#include <Eigen/Dense>

ros::Publisher pub;
double color_r;
double color_g;
double color_b;

void write_marker(visualization_msgs::Marker& marker, const strands_perception_msgs::Table& table)
{
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.pose.position.x = 0.0;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.0;
    Eigen::Quaterniond quat;
    // these markers are in the camera's frame of reference
    quat.setIdentity();
    marker.pose.orientation.x = quat.x();
    marker.pose.orientation.y = quat.y();
    marker.pose.orientation.z = quat.z();
    marker.pose.orientation.w = quat.w();
    marker.scale.x = 0.02;
    int n = table.tabletop.points.size();
    marker.points.resize(2*n);
    for (int i = 0; i < n-1 ; ++i) {
        marker.points[2*i+2].x = table.tabletop.points[i].x;
        marker.points[2*i+2].y = table.tabletop.points[i].y;
        marker.points[2*i+2].z = table.tabletop.points[i].z;
        marker.points[2*i+3].x = table.tabletop.points[i+1].x;
        marker.points[2*i+3].y = table.tabletop.points[i+1].y;
        marker.points[2*i+3].z = table.tabletop.points[i+1].z;
    }
    marker.points[0].x = table.tabletop.points[n-1].x;
    marker.points[0].y = table.tabletop.points[n-1].y;
    marker.points[0].z = table.tabletop.points[n-1].z;
    marker.points[1].x = table.tabletop.points[0].x;
    marker.points[1].y = table.tabletop.points[0].y;
    marker.points[1].z = table.tabletop.points[0].z;
    
    marker.color.a = 1.0;
    marker.color.r = color_r;
    marker.color.g = color_g;
    marker.color.b = color_b;
    
}

void callback(const strands_perception_msgs::Table::ConstPtr& msg)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = msg->header.frame_id;
    marker.header.stamp = ros::Time();
    marker.ns = "my_namespace"; // what's this for?
    marker.id = 1; // solve this later by looking at database, table_id
    marker.action = visualization_msgs::Marker::ADD;
    pub.publish(marker);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "table_to_marker");
    ros::NodeHandle n;
    
    // Initialize node parameters from launch file or command line.
    // Use a private node handle so that multiple instances of the node can be run simultaneously
    // while using different parameters.
	ros::NodeHandle pn("~");
	std::string input;
	pn.param<std::string>("input", input, std::string("/table_tables"));
	std::string output;
	pn.param<std::string>("output", output, std::string("/table_markers"));
	pn.param<double>("color_r", color_r, 0.0);
	pn.param<double>("color_g", color_g, 1.0);
	pn.param<double>("color_b", color_b, 0.0);
	
	ros::Subscriber sub = n.subscribe(input, 1, callback);
	pub = n.advertise<visualization_msgs::Marker>(output, 1);
    
    ros::spin();
    
    return 0;
}

