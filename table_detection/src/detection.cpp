#include <iostream>

#include "ros/ros.h"

#include "primitive_extraction/primitive.h"
#include "primitive_extraction/primitive_array.h"

#include <Eigen/Dense>

ros::Publisher pub;
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
    for (int i = 1; i < hull.size()-1; ++i) {
        Eigen::Vector2d p1(hull[i].x, hull[i].y);
        Eigen::Vector2d p2(hull[i+1].x, hull[i+1].y);
        // calculate area by Heron's formula
        double a = (p1-p0).norm();
        double b = (p2-p0).norm();
        double c = (p2-p1).norm();
        double s = 0.5*(a + b + c);
        area += sqrt(s*(s-a)*(s-b)*(s-c));
    }
    return area;
}

void callback(const primitive_extraction::primitive_array::ConstPtr& msg)
{
    
    std::cout << "Tjena!" << std::endl;
    int n = msg->primitives.size();
    primitive_extraction::primitive_array tables;
    tables.camera_frame = msg->camera_frame;
    
    for (int i = 0; i < n; ++i) {
        primitive_extraction::primitive p = msg->primitives[i];
        
        /*std::cout << p.params[2] << std::endl;
        std::cout << p.params[3] << std::endl;
        std::cout << p.params[4] << std::endl;
        
        Eigen::Vector3d normal(p.params[2], p.params[3], p.params[4]);
        normal.normalize();
        std::cout << normal(2) << std::endl;*/
        
        // check normal
        double alpha = acos(fabs(p.params[4]));
        if (alpha > max_angle) {
            std::cout << "Stopped because of angle: " << alpha << std::endl;
            continue;
        }
        
        double camera_height = 0.0;
        // check height
        double height = camera_height + p.pose.position.z;
        if (height < min_height || height > max_height) {
            std::cout << "Stopped because of height: " << height << std::endl;
            continue;
        }
        
        // check size
        double area = compute_plane_area(p.points);
        if (area < min_area) {
            std::cout << "Stopped because of area: " << area << std::endl;
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
        //double ratio = p.params[0] > p.params[1] ? p.params[1]/p.params[0] : p.params[0]/p.params[1]; 
        if (ratio < min_side_ratio) {
            std::cout << "Stopped because of ratio: " << ratio << std::endl;
            continue;
        }
        
        std::cout << "This one got through!" << std::endl;
        
        tables.primitives.push_back(p);
    }
    
    pub.publish(tables);
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
	pn.param<std::string>("input", input, std::string("/primitives"));
	ros::Subscriber sub = n.subscribe(input, 1, callback);
	std::string output;
	pn.param<std::string>("output", output, std::string("/tables"));
	pub = n.advertise<primitive_extraction::primitive_array>(output, 1);
    
    ros::spin();
    
    return 0;
}

