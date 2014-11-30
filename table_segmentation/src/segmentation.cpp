#include "mongodb_store/message_store.h"
#include "ros/ros.h"
#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <tf/transform_listener.h>

#include "strands_perception_msgs/Table.h"
#include "table_segmentation/SegmentTable.h"

using namespace mongodb_store;
using namespace Eigen;

MessageStoreProxy* messageStore;
tf::TransformListener* listener;
double scaling;

// compute the enclosing box from a set of points defining the convex hull
void find_smallest_enclosing_box(Vector2f& cmin, Matrix2f& axes,
                                 Vector2f& lengths, std::vector<geometry_msgs::Point32>& pts)
{
    // the idea is to iterate through all the sides, take the side to be one
    // side of a rectangle, project the points on the side and see what area
    // we get with this rectangle. the smallest one will be returned.
    // the smallest enclosing box should always have one side co-linear
    // with one side of the convex hull.
    std::vector<Vector2f, aligned_allocator<Vector2f> > dpts;
    dpts.resize(pts.size());
    for (size_t i = 0; i < pts.size(); ++i) {
        dpts[i] = Vector2f(double(pts[i].x), double(pts[i].y));
    }
    double areamin = INFINITY;
    for (size_t i = 0; i < dpts.size(); ++i) { // all lines, find smallest area
        Vector2f vec = dpts[(i+1)%int(dpts.size())] - dpts[i];
        Vector2f ovec(-vec(1), vec(0));
        vec.normalize();
        ovec.normalize();
        double widthmin = INFINITY;
        double widthmax = -INFINITY;
        double heightmax = 0;
        for (size_t j = 0; j < dpts.size(); ++j) { // find width and height
            double proj = vec.dot(dpts[j] - dpts[i]);
            double oproj = ovec.dot(dpts[j] - dpts[i]);
            if (proj < widthmin) {
                widthmin = proj;
            }
            if (proj > widthmax) {
                widthmax = proj;
            }
            if (fabs(oproj) > fabs(heightmax)) {
                heightmax = oproj;
            }
        }
        double width = (widthmax - widthmin);
        double area = fabs(heightmax)*width;
        if (area < areamin) {
            areamin = area;
            axes.col(0) = vec;
            axes.col(1) = ovec;
            cmin = dpts[i] + 0.5*(widthmin*vec + widthmax*vec + heightmax*ovec);
            lengths = Vector2f(width, heightmax);
        }
    }
}

bool service_callback(table_segmentation::SegmentTable::Request& req,
                      table_segmentation::SegmentTable::Response& res) // primitive_array input, table_ids return
{
    std::vector<boost::shared_ptr<strands_perception_msgs::Table> > results;
    std::string name = req.table_id;
    //Get it back, by default get one
    if (!messageStore->queryNamed<strands_perception_msgs::Table>(name, results)) {
        ROS_ERROR("Couldn't find table %s in datacentre.", name.c_str());
    }
    Vector2f cmin; Vector2f cmax; Matrix2f axes; Vector2f lengths;
    find_smallest_enclosing_box(cmin, axes, lengths, results[0]->tabletop.points);
    float height = results[0]->pose.pose.position.z;
    float zmin = height - 0.2;
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr msg_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::fromROSMsg(req.cloud, *msg_cloud);
    size_t n = msg_cloud->points.size();
    
       
    Matrix3f R;
    Vector3f t;
	if (req.transform.size() == 16) { // we got a transform supplied already
	    Eigen::Matrix<float, 4, 4, Eigen::RowMajor> transform;
		std::cout << "A transform was supplied, not using ROS TF" << std::endl;
		for (unsigned int i=0;i<16;i++) {
		  *(transform.data()+i) = req.transform[i];
		}		
		//basis = transform.topLeftCorner<3,3>();
		R = transform.block<3,3>(0,0);
		t = transform.block<3,1>(0,3);
		std::cout << R << std::endl;
		std::cout << t << std::endl;
	} else {
		std::string dest_frame = "/map";
		tf::StampedTransform transform;
		try {
			//listener->transformPointCloud("/head_xtion_rgb_optical_frame", msg->header.stamp, *msg, msg->header.frame_id, cout);
			listener->lookupTransform(dest_frame, req.cloud.header.frame_id, req.cloud.header.stamp, transform);
		}
		catch (tf::TransformException ex) {
			ROS_INFO("%s",ex.what());
			return false;
		}
	 
		tf::Vector3 origin = transform.getOrigin();
		tf::Matrix3x3 basis = transform.getBasis();
		for (size_t i = 0; i < 3; ++i) {
			t(i) = origin.m_floats[i];
			for (size_t j = 0; j < 3; ++j) {
				R(i, j) = basis.getRow(i).m_floats[j];
			}
		}
    }
	
    
    Vector3f point;
    Vector2f point2d;
    float lx, ly;
    lengths = float(scaling)*lengths;
    for (size_t i = 0; i < n; ++i) {
        // transform point
        point = R*msg_cloud->points[i].getVector3fMap() + t;
        point2d = point.segment<2>(0) - cmin;
        lx = point2d.dot(axes.col(0));
        ly = point2d.dot(axes.col(1));
        // check if inbetween certain heights and in table rectangle
        if (lx < -lengths(0)/2.0 || lx > lengths(0)/2.0 || ly < -lengths(1)/2.0 || ly > lengths(1)/2.0 || point(2) < zmin) {
            msg_cloud->points[i].x = INFINITY;
            msg_cloud->points[i].y = INFINITY;
            msg_cloud->points[i].z = INFINITY;
        }
    }
    
    pcl::toROSMsg(*msg_cloud, res.cloud);
    res.cloud.header = req.cloud.header;
    
    return true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "segmentation");
    ros::NodeHandle n;
    
    ros::NodeHandle pn("~");
	std::string input;
	pn.param<double>("scaling", scaling, 0.9);

    //Create object which does the work for us.
    messageStore = new MessageStoreProxy(n, "frozen_tables");
    listener = new tf::TransformListener();
    
    ros::ServiceServer service = n.advertiseService("segment_table", &service_callback);
    
    ros::spin();
    
    return 0;
}
