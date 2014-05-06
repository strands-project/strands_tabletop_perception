#include <iostream>

#include "ros/ros.h"

#include "primitive_extraction/Primitive.h"
#include "primitive_extraction/PrimitiveArray.h"
#include "strands_perception_msgs/Table.h"
#include "table_tracking.h"
#include "primitives_to_tables/PrimitivesToTables.h"

#include <Eigen/Dense>

ros::Publisher pub;
boost::shared_ptr<table_tracking> t; // keeps track of all tables
//std::auto_ptr??

void create_tables(std::vector<strands_perception_msgs::Table>& tables,
                   const primitive_extraction::PrimitiveArray& primitives)
{
    tables.resize(primitives.primitives.size());
    // convert the primitive types into tables
    for (size_t i = 0; i < primitives.primitives.size(); ++i) {
        strands_perception_msgs::Table table;
        primitive_extraction::Primitive primitive = primitives.primitives[i];
        table.header.frame_id = primitives.camera_frame;
        table.pose.pose = primitive.pose;
        table.tabletop.points.resize(primitive.points.size());
        for (size_t j = 0; j < primitive.points.size(); ++j) {
            table.tabletop.points[j].x = primitive.points[j].x;
            table.tabletop.points[j].y = primitive.points[j].y;
            table.tabletop.points[j].z = primitive.points[j].z;
        }
        // check overlap, possibly merging with previous tables
        //t->add_detected_table(table);
        // if merged, that will be the table published
        //pub.publish(table); // always publish detected tables
        tables[i] = table;
    }
    
    t->add_detected_tables(tables);
    for (size_t i = 0; i < tables.size(); ++i) {
        pub.publish(tables[i]);
    }
}

void callback(const primitive_extraction::PrimitiveArray::ConstPtr& msg)
{
    std::vector<strands_perception_msgs::Table> tables;
    create_tables(tables, *msg);
}

bool service_callback(primitives_to_tables::PrimitivesToTables::Request& req,
                      primitives_to_tables::PrimitivesToTables::Response& res) // primitive_array input, table_ids return
{
    create_tables(res.tables, req.primitives);
    return true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "primitives_to_tables");
    ros::NodeHandle n;
    t = boost::shared_ptr<table_tracking>(new table_tracking(n));
    
    ros::NodeHandle pn("~");
	std::string input;
	pn.param<std::string>("input", input, std::string("/table_detection/table_primitives"));
	ros::Subscriber sub = n.subscribe(input, 1, callback);
	std::string output;
	pn.param<std::string>("output", output, std::string("/table_detection/tables"));
	pub = n.advertise<strands_perception_msgs::Table>(output, 1);
	
	ros::ServiceServer service = n.advertiseService("primitives_to_tables", &service_callback);
    
    ros::spin();
    
    return 0;
}

