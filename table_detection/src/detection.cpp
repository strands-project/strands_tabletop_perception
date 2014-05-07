#include <iostream>

#include "ros/ros.h"

#include "primitive_extraction/Primitive.h"
#include "primitive_extraction/PrimitiveArray.h"
#include "primitives_to_tables/PrimitivesToTables.h"
#include "table_detection/DetectTables.h"
#include "primitive_extraction/ExtractPrimitives.h"

#include <Eigen/Dense>

ros::Publisher pub;
ros::ServiceClient table_client;
ros::ServiceClient primitive_client;
double min_height;
double max_height;
double max_angle;
double min_side_ratio;
double min_area;

double compute_plane_area(std::vector<geometry_msgs::Point>& hull)
{
    double area = 0.0;
    // assume that it's almost parallell to the ground
    Eigen::Vector2d p0(hull[0].x, hull[0].y);
    for (size_t i = 1; i < hull.size()-1; ++i) {
        Eigen::Vector2d p1(hull[i].x, hull[i].y);
        Eigen::Vector2d p2(hull[i+1].x, hull[i+1].y);
        // calculate area by Heron's formula
        // this can be done by det as in table_tracking
        double a = (p1-p0).norm();
        double b = (p2-p0).norm();
        double c = (p2-p1).norm();
        double s = 0.5*(a + b + c);
        area += sqrt(s*(s-a)*(s-b)*(s-c));
    }
    return area;
}

void detect_tables(primitive_extraction::PrimitiveArray& tables,
                   std::vector<size_t>& indices,
                   const primitive_extraction::PrimitiveArray& primitives)
{
    tables.camera_frame = primitives.camera_frame;
    size_t n = primitives.primitives.size();
    
    for (size_t i = 0; i < n; ++i) {
        primitive_extraction::Primitive p = primitives.primitives[i];
        
        // check normal
        double alpha = acos(fabs(p.params[4]));
        if (alpha > max_angle) {
            //ROS_INFO("Stopped because of angle: %f", alpha);
            continue;
        }
        
        double camera_height = 0.0;
        // check height
        double height = camera_height + p.pose.position.z;
        if (height < min_height || height > max_height) {
            //ROS_INFO("Stopped because of height: %f", height);
            continue;
        }
        
        // check size
        double area = compute_plane_area(p.points);
        if (area < min_area) {
            //ROS_INFO("Stopped because of area: %f", area);
            continue;
        }
        
        // check shape
        double minside, maxside;
        if (p.params[0] > p.params[1]) {
            minside = p.params[1];
            maxside = p.params[0];
        }
        else {
            minside = p.params[0];
            maxside = p.params[1];     
        }
        
        double ratio = minside/maxside;
        if (ratio < min_side_ratio) {
            //ROS_INFO("Stopped because of ratio: %f", ratio);
            continue;
        }
        
        tables.primitives.push_back(p);
        indices.push_back(i);
    }
}

void callback(const primitive_extraction::PrimitiveArray::ConstPtr& msg)
{
    primitive_extraction::PrimitiveArray tables;
    std::vector<size_t> indices;
    detect_tables(tables, indices, *msg);
    pub.publish(tables);
}

bool service_callback(table_detection::DetectTables::Request& req,
                      table_detection::DetectTables::Response& res) // primitive_array input, table_ids return
{
    // get the primitives through service call
    primitive_extraction::ExtractPrimitives primitive_srv;
    primitive_srv.request.pointcloud = req.pointcloud;
    if (!primitive_client.call(primitive_srv)) {
        ROS_ERROR("Failed to call primitive service.");
    }
    
    // detect the tables among the primitives
    primitive_extraction::PrimitiveArray table_primitives;
    std::vector<size_t> indices;
    detect_tables(table_primitives, indices, primitive_srv.response.primitives);
    
    // call service to add to database
    primitives_to_tables::PrimitivesToTables table_srv;
    table_srv.request.primitives = table_primitives;
    if (!table_client.call(table_srv)) {
        ROS_ERROR("Failed to call table service.");
    }
    
    // assign the results
    res.tables = table_srv.response.tables;
    res.indices.resize(table_srv.response.tables.size());
    for (size_t i = 0; i < table_srv.response.tables.size(); ++i) {
        res.indices[i] = primitive_srv.response.indices[indices[i]];
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "detection");
    ros::NodeHandle n;
    
    ros::NodeHandle pn("~");
    pn.param<double>("min_height", min_height, 0.55);
    pn.param<double>("max_height", max_height, 1.1);
    pn.param<double>("max_angle", max_angle, 0.314);
    pn.param<double>("min_side_ration", min_side_ratio, 0.25);
    pn.param<double>("min_area", min_area, 0.3);
	
	std::string input;
	pn.param<std::string>("input", input, std::string("/primitive_extraction/primitives"));
	ros::Subscriber sub = n.subscribe(input, 1, callback);
	std::string output;
	pn.param<std::string>("output", output, std::string("/table_detection/table_primitives"));
	pub = n.advertise<primitive_extraction::PrimitiveArray>(output, 1);
	ros::ServiceServer service = n.advertiseService("detect_tables", &service_callback);
	table_client = n.serviceClient<primitives_to_tables::PrimitivesToTables>("primitives_to_tables");
	primitive_client = n.serviceClient<primitive_extraction::ExtractPrimitives>("extract_primitives");
    
    ros::spin();
    
    return 0;
}

