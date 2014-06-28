#include "ros/ros.h"
#include "table_segmentation/SegmentTable.h"

ros::Publisher pub;
ros::ServiceClient segmentation_client;

void callback(const sensor_msgs::PointCloud2::ConstPtr& msg) // primitive_array input, table_ids return
{
    table_segmentation::SegmentTable segmentation_srv;
    segmentation_srv.request.cloud = *msg;
    segmentation_srv.request.table_id = "table1";
    if (!segmentation_client.call(segmentation_srv)) {
        ROS_ERROR("Failed to call segmentation service.");
        return;
    }
    ROS_INFO("Publishing point cloud.");
    pub.publish(segmentation_srv.response.cloud);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_segmentation");
    ros::NodeHandle n;
    
    ros::NodeHandle pn("~");
	std::string input;
	pn.param<std::string>("input", input, std::string("/head_xtion/depth_registered/points"));
	ros::Subscriber sub = n.subscribe(input, 1, callback);
	std::string output;
	pn.param<std::string>("output", output, std::string("/table_segmentation/cloud"));
	pub = n.advertise<sensor_msgs::PointCloud2>(output, 1);
	
	segmentation_client = n.serviceClient<table_segmentation::SegmentTable>("segment_table");
    
    ros::spin();
    
    return 0;
}

