#ifndef TABLE_TRACKING_H
#define TABLE_TRACKING_H

#include "ros/ros.h"
#include <Eigen/Dense>

#include "strands_perception_msgs/Table.h"
#include "ros_datacentre/message_store.h"

class table_tracking {
private:
    ros_datacentre::MessageStoreProxy message_store; // for storing in the datacentre
    // tables in datacentre
    std::vector<boost::shared_ptr<strands_perception_msgs::Table> > tables;
    // the mass centers of the tables in the datacenter
    //std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > centers;
    //void compute_center_of_mass(Eigen::Vector2d& mid, const geometry_msgs::Polygon& p);
    void compute_center_of_mass(strands_perception_msgs::Table& table);
    //bool are_overlapping(Eigen::Vector2d& mida, const geometry_msgs::Polygon& a, Eigen::Vector2d& midb, const geometry_msgs::Polygon& b);
    //bool are_overlapping(strands_perception_msgs::Table& a, strands_perception_msgs::Table& b);
    bool are_overlapping(strands_perception_msgs::Table& a, strands_perception_msgs::Table& b);
    //bool center_contained(const geometry_msgs::Polygon& p, const Eigen::Vector2d& c);
    bool center_contained(const geometry_msgs::Polygon& p, const geometry_msgs::PoseWithCovariance& mid);
    int find_next_point(const Eigen::Vector2d& q, const Eigen::Vector2d& c, const std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> >& p, std::vector<int>& used);
    //void union_convex_hull(geometry_msgs::Polygon& res, const Eigen::Vector2d& mida, const geometry_msgs::Polygon& a, const Eigen::Vector2d& midb, const geometry_msgs::Polygon& b);
    void union_convex_hull(geometry_msgs::Polygon& res,
                           const geometry_msgs::PoseWithCovariance& ca, const geometry_msgs::Polygon& a,
                           const geometry_msgs::PoseWithCovariance& cb, const geometry_msgs::Polygon& b);
    void convex_hull(geometry_msgs::Polygon& res, const Eigen::Vector2d& c, const std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> >& p);
    //void rectify_orientation(const Eigen::Vector2d& c, geometry_msgs::Polygon& p);
    void rectify_orientation(strands_perception_msgs::Table& table);
public:
    void add_detected_tables(std::vector<strands_perception_msgs::Table>& t);
    bool add_detected_table(strands_perception_msgs::Table& table);
    table_tracking(ros::NodeHandle& n);
};
#endif // TABLE_TRACKING_H
