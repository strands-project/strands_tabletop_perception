#include "table_tracking.h"

#include <boost/lexical_cast.hpp>
#include "octave_convenience.h"

using namespace Eigen;

table_tracking::table_tracking(ros::NodeHandle& n) : message_store(n, "tables")
{
    message_store.query<strands_perception_msgs::Table>(tables);
    //Vector2d mid;
    //centers.reserve(tables.size());
    //for (size_t i = 0; i < tables.size(); ++i) {
        //compute_center_of_mass(mid, tables[i]->tabletop);
        //centers.push_back(mid);
    //}
    ROS_INFO("Found %lu tables in database.", tables.size());
}

// compute the center of mass of a convex polygon message
// TODO: do this in 3d instead
/*void table_tracking::compute_center_of_mass(Vector2d& mid, const geometry_msgs::Polygon& p)
{
    Eigen::Vector2d p0(p.points[0].x, p.points[0].y);
    Matrix2d D;
    Vector2d mid_triangle;
    mid = Vector2d(0, 0);
    double area_triangle;
    double area = 0;
    for (size_t i = 1; i < p.points.size()-1; ++i) {
        Vector2d p1(p.points[i].x, p.points[i].y);
        Vector2d p2(p.points[i+1].x, p.points[i+1].y);
        mid_triangle = 1.0/3.0*(p0 + p1 + p2);
        D.col(0) = p1 - p0;
        D.col(1) = p2 - p0;
        area_triangle = 0.5*D.determinant();
        mid += area_triangle*mid_triangle;
        area += area_triangle;
    }
    mid = 1.0/area*mid;
}*/

void table_tracking::compute_center_of_mass(strands_perception_msgs::Table& table)
{
    geometry_msgs::Polygon p = table.tabletop;
    Eigen::Vector2d p0(p.points[0].x, p.points[0].y);
    Matrix2d D;
    Vector2d mid_triangle;
    Vector2d mid(0.0, 0.0);
    double area_triangle;
    double area = 0;
    double height = p.points[0].z;
    for (size_t i = 1; i < p.points.size()-1; ++i) {
        Vector2d p1(p.points[i].x, p.points[i].y);
        Vector2d p2(p.points[i+1].x, p.points[i+1].y);
        mid_triangle = 1.0/3.0*(p0 + p1 + p2);
        D.col(0) = p1 - p0;
        D.col(1) = p2 - p0;
        area_triangle = 0.5*D.determinant();
        mid += area_triangle*mid_triangle;
        area += area_triangle;
        height += p.points[i].z;
    }
    mid = 1.0/area*mid;
    height += p.points[p.points.size()-1].z;
    height /= double(p.points.size());
    
    table.pose.pose.position.x = mid(0);
    table.pose.pose.position.y = mid(1);
    table.pose.pose.position.z = height;
}


/*bool table_tracking::center_contained(const geometry_msgs::Polygon& p, const Vector2d& c)
{
    size_t n = p.points.size();
    double is_signed;
    
    // check if c is on the same side of each line
    for (size_t i = 0; i < n; ++i) {
        Vector2d p0(p.points[i].x, p.points[i].y);
        Vector2d p1(p.points[(i+1)%n].x, p.points[(i+1)%n].y);
        Vector2d v0 = p1 - p0;
        Vector2d v1(-v0(1), v0(0)); // rotate line segment
        Vector2d d = c - p0;
        double prod = d.dot(v1);
        if (i == 0) {
            is_signed = prod;
            continue;
        }
        if (is_signed*prod < 0) {
            return false;
        }
    }
    return true;
}*/

bool table_tracking::center_contained(const geometry_msgs::Polygon& p, const geometry_msgs::PoseWithCovariance& mid)
{
    Vector2d c(mid.pose.position.x, mid.pose.position.y);
    size_t n = p.points.size();
    double is_signed;
    
    // check if c is on the same side of each line
    for (size_t i = 0; i < n; ++i) {
        Vector2d p0(p.points[i].x, p.points[i].y);
        Vector2d p1(p.points[(i+1)%n].x, p.points[(i+1)%n].y);
        Vector2d v0 = p1 - p0;
        Vector2d v1(-v0(1), v0(0)); // rotate line segment
        Vector2d d = c - p0;
        double prod = d.dot(v1);
        if (i == 0) {
            is_signed = prod;
            continue;
        }
        if (is_signed*prod < 0) {
            return false;
        }
    }
    return true;
}

// check if two tables are overlapping
/*bool table_tracking::are_overlapping(Vector2d& mida, const geometry_msgs::Polygon& a, Vector2d& midb, const geometry_msgs::Polygon& b)
{   
    return (center_contained(a, midb) || center_contained(b, mida));
}*/

bool table_tracking::are_overlapping(strands_perception_msgs::Table& a, strands_perception_msgs::Table& b)
{   
    return (center_contained(a.tabletop, b.pose) || center_contained(b.tabletop, a.pose));
}

// make sure the centre is always on the left(?) side of the line
// this should not be needed in the algorithms
/*void table_tracking::rectify_orientation(const Vector2d& c, geometry_msgs::Polygon& p)
{
    Vector2d p0(p.points[0].x, p.points[0].y);
    Vector2d p1(p.points[1].x, p.points[1].y);
    Vector2d v = p1 - p0;
    Vector2d o(-v(1), v(0));
    if (o.dot(c - p0) < 0) {
        std::reverse(p.points.begin(), p.points.end());
    }
}*/

void table_tracking::rectify_orientation(strands_perception_msgs::Table& table)
{
    geometry_msgs::Polygon p = table.tabletop;
    Vector2d c(table.pose.pose.position.x, table.pose.pose.position.y);
    Vector2d p0(p.points[0].x, p.points[0].y);
    Vector2d p1(p.points[1].x, p.points[1].y);
    Vector2d v = p1 - p0;
    Vector2d o(-v(1), v(0));
    if (o.dot(c - p0) < 0) {
        std::reverse(p.points.begin(), p.points.end());
    }
}


// compute the convex union of two convex hulls by simply
// computing a new convex hull for the union of point vertices
/*void table_tracking::union_convex_hull(geometry_msgs::Polygon& res, const Vector2d& mida, const geometry_msgs::Polygon& a, const Vector2d& midb, const geometry_msgs::Polygon& b)
{
    std::vector<Vector2d, aligned_allocator<Vector2d> > p;
    p.resize(a.points.size() + b.points.size());
    size_t n = a.points.size();
    double zmean = 0;
    for (size_t i = 0; i < n; ++i) {
        p[i] = Vector2d(a.points[i].x, a.points[i].y);
        zmean += a.points[i].z;
    }
    for (size_t i = 0; i < b.points.size(); ++i) {
        p[n + i] = Vector2d(b.points[i].x, b.points[i].y);
        zmean += b.points[i].z;
    }
    convex_hull(res, mida, p);
    zmean /= double(p.size());
    for (size_t i = 0; i < res.points.size(); ++i) {
        res.points[i].z = zmean;
    }
}*/

void table_tracking::union_convex_hull(geometry_msgs::Polygon& res,
                                       const geometry_msgs::PoseWithCovariance& ca, const geometry_msgs::Polygon& a,
                                       const geometry_msgs::PoseWithCovariance& cb, const geometry_msgs::Polygon& b)
{
    Vector2d mida(ca.pose.position.x, ca.pose.position.y);
    Vector2d midb(cb.pose.position.x, cb.pose.position.y);
    
    std::vector<Vector2d, aligned_allocator<Vector2d> > p;
    p.resize(a.points.size() + b.points.size());
    size_t n = a.points.size();
    double zmean = 0;
    for (size_t i = 0; i < n; ++i) {
        p[i] = Vector2d(a.points[i].x, a.points[i].y);
        zmean += a.points[i].z;
    }
    for (size_t i = 0; i < b.points.size(); ++i) {
        p[n + i] = Vector2d(b.points[i].x, b.points[i].y);
        zmean += b.points[i].z;
    }
    convex_hull(res, mida, p);
    zmean /= double(p.size());
    for (size_t i = 0; i < res.points.size(); ++i) {
        res.points[i].z = zmean;
    }
    
    /*std::vector<double> ax, ay;
    ax.resize(a.points.size());
    ay.resize(a.points.size());
    for (int i = 0; i < a.points.size(); ++i) {
        ax[i] = a.points[i].x;
        ay[i] = a.points[i].y;
    }
    std::vector<double> bx, by;
    bx.resize(b.points.size());
    by.resize(b.points.size());
    for (int i = 0; i < b.points.size(); ++i) {
        bx[i] = b.points[i].x;
        by[i] = b.points[i].y;
    }
    std::vector<double> px, py;
    px.resize(res.points.size());
    py.resize(res.points.size());
    for (int i = 0; i < res.points.size(); ++i) {
        px[i] = res.points[i].x;
        py[i] = res.points[i].y;
    }
    octave_convenience o;
    o << "plot(";
    o.append_vector(ax);
    o << ", ";
    o.append_vector(ay);
    o << ", 'b'); hold on; plot(";
    o.append_vector(bx);
    o << ", ";
    o.append_vector(by);
    o << ", 'r');";
    o << "plot(";
    o.append_vector(px);
    o << ", ";
    o.append_vector(py);
    o << ", 'g');";
    o << "plot(" << mida(0) << ", " << mida(1) << ", 'bo');";
    o << "plot(" << midb(0) << ", " << midb(1) << ", 'ro'); axis equal; pause";
    o.eval();*/
}

// compute the convex hull encompassing a set of points
void table_tracking::convex_hull(geometry_msgs::Polygon& res, const Vector2d& c, const std::vector<Vector2d, aligned_allocator<Vector2d> >& p)
{
    geometry_msgs::Point32 p32;
    std::vector<int> used;
    used.resize(p.size(), 0);
    
    int first_ind;
    int previous_ind;
    for (size_t i = 0; i < p.size(); ++i) {
        int ind = find_next_point(p[i], c, p, used);
        if (ind != -1) {
            used[i] = 1;
            used[ind] = 1;
            first_ind = i;
            previous_ind = ind;
            break;
        }
        used[i] = 1;
    }
    p32.x = p[first_ind](0);
    p32.y = p[first_ind](1);
    res.points.push_back(p32);
    while (previous_ind != first_ind) {
        p32.x = p[previous_ind](0);
        p32.y = p[previous_ind](1);
        res.points.push_back(p32);
        int ind = find_next_point(p[previous_ind], c, p, used);
        previous_ind = ind;
        used[first_ind] = 0;
    }
}

int table_tracking::find_next_point(const Vector2d& q, const Vector2d& c, const std::vector<Vector2d, aligned_allocator<Vector2d> >& p, std::vector<int>& used)
{
    Vector2d p0;
    Vector2d p1;
    Vector2d v;
    Vector2d o;
    bool found;
    for (size_t j = 0; j < p.size(); ++j) {
        if (used[j]) {
            continue;
        }
        p0 = q;
        p1 = p[j];
        if (p0(0) == p1(0) && p0(1) == p1(1)) {
            continue;
        }
        v = p1 - p0;
        o = Vector2d(-v(1), v(0));
        if (o.dot(c - p0) < 0) {
            continue;
        }
        found = true;
        for (size_t k = 0; k < p.size(); ++k) {
            if (k == j || (p[k](0) == q(0) && p[k](1) == q(1))) {
                continue;
            }
            if (o.dot(p[k] - p0) < 0) {
                found = false;
                break;
            }
        }
        if (found) {
            used[j] = 1;
            return j;
        }
    }
    return -1;
}

void table_tracking::add_detected_tables(std::vector<strands_perception_msgs::Table>& t)
{
    tables.clear();
    message_store.query<strands_perception_msgs::Table>(tables);
    for (size_t i = 0; i < t.size(); ++i) {
        add_detected_table(t[i]);
    }
}

std::string table_tracking::new_table_id()
{
    int maxval = 0;
    for (size_t i = 0; i < tables.size(); ++i) {
        int val = boost::lexical_cast<int>(tables[i]->table_id.substr(5));
        if (val > maxval) {
            maxval = val;
        }
    }
    return boost::lexical_cast<std::string>(maxval+1);
}

// add a detected table into the database, either by identifying it with a
// previous table or by creating a new table instance. tables are merged
// if they are overlapping, incoming message is modified to reflect update
bool table_tracking::add_detected_table(strands_perception_msgs::Table& table)
{
    ROS_INFO("Got a new table, number of tables: %lu", tables.size());
    //Vector2d mid;
    //compute_center_of_mass(mid, table.tabletop);
    compute_center_of_mass(table);
    //rectify_orientation(mid, table.tabletop);
    rectify_orientation(table);
    for (size_t i = 0; i < tables.size(); ++i) {
        //if (!are_overlapping(tables[i]->pose, tables[i]->tabletop, table.pose, table.tabletop)) {
        if (!are_overlapping(*tables[i], table)) {
            continue;
        }
        geometry_msgs::Polygon p;
        //union_convex_hull(p, mid, table.tabletop, centers[i], tables[i]->tabletop);
        union_convex_hull(p, table.pose, table.tabletop, tables[i]->pose, tables[i]->tabletop);
        tables[i]->tabletop = p;
        //compute_center_of_mass(mid, table.tabletop);
        compute_center_of_mass(*tables[i]);
        //centers[i] = mid;
        message_store.updateNamed(std::string("table") + tables[i]->table_id, *tables[i]);
        
        table = *tables[i]; // make sure that the published table has the same value
        return false;
    }
    //centers.push_back(mid);
    //table.table_id = std::string("table") + boost::lexical_cast<std::string>(tables.size());
    table.table_id = std::string("table") + new_table_id();
    tables.push_back(boost::shared_ptr<strands_perception_msgs::Table>(new strands_perception_msgs::Table(table)));
    //Insert something with a name
    message_store.insertNamed(table.table_id, table);
    return true;
}
