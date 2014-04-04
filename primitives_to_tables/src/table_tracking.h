#ifndef TABLE_TRACKING_H
#define TABLE_TRACKING_H

#include "ros/ros.h"
#include <Eigen/Dense>

#include "strands_perception_msgs/Table.h"
#include "ros_datacentre/message_store.h"

class table_tracking {
private:
    ros_datacentre::MessageStoreProxy message_store;
    std::vector<boost::shared_ptr<strands_perception_msgs::Table> > tables;
    std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > centers;
    std::vector<double> radia;
    void compute_center_of_mass(Eigen::Vector2d& mid, const geometry_msgs::Polygon& p);
    bool are_overlapping(Eigen::Vector2d& mida, const geometry_msgs::Polygon& a, Eigen::Vector2d& midb, const geometry_msgs::Polygon& b);
    bool center_contained(const geometry_msgs::Polygon& p, const Eigen::Vector2d& c);
    int find_next_point(const Eigen::Vector2d& q, const Eigen::Vector2d& c, std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> >& p, std::vector<int>& used, bool& reverse, bool first = false);
    void union_convex_hull(geometry_msgs::Polygon& res, const Eigen::Vector2d& mida, const geometry_msgs::Polygon& a, const Eigen::Vector2d& midb, const geometry_msgs::Polygon& b);
    void rectify_orientation(const Eigen::Vector2d& c, geometry_msgs::Polygon& p);
public:
    bool add_detected_table(strands_perception_msgs::Table& table);
    table_tracking(ros::NodeHandle& n);
};
#endif // TABLE_TRACKING_H
