#include "table_tracking.h"

#include <boost/lexical_cast.hpp>
#include "octave_convenience.h"

using namespace Eigen;

table_tracking::table_tracking(ros::NodeHandle& n) : message_store(n)
{
    std::cout << "1" << std::endl;
    //message_store.query<strands_perception_msgs::Table>(tables);
    std::cout << "2" << std::endl;
    Vector2d mid;
    centers.reserve(tables.size());
    for (int i = 0; i < tables.size(); ++i) {
        compute_center_of_mass(mid, tables[i]->tabletop);
        centers.push_back(mid);
    }
    std::cout << "Found " << tables.size() << " tables" << std::endl;
}

void table_tracking::compute_center_of_mass(Vector2d& mid, const geometry_msgs::Polygon& p)
{
    /*std::vector<double> px, py;
    px.resize(p.points.size());
    py.resize(p.points.size());
    for (int i = 0; i < p.points.size(); ++i) {
        px[i] = p.points[i].x;
        py[i] = p.points[i].y;
    }
    
    octave_convenience o;
    o << "plot(";
    o.append_vector(px);
    o << ", ";
    o.append_vector(py);
    o << ", 'b'); axis equal; hold on;";*/

    Eigen::Vector2d p0(p.points[0].x, p.points[0].y);
    Matrix2d D;
    Vector2d mid_triangle;
    mid = Vector2d(0, 0);
    double area_triangle;
    double area = 0;
    for (int i = 1; i < p.points.size()-1; ++i) {
        Vector2d p1(p.points[i].x, p.points[i].y);
        Vector2d p2(p.points[i+1].x, p.points[i+1].y);
        mid_triangle = 1.0/3.0*(p0 + p1 + p2);
        D.col(0) = p1 - p0;
        D.col(1) = p2 - p0;
        area_triangle = 0.5*D.determinant();
        mid += area_triangle*mid_triangle;
        area += area_triangle;
        //o << "plot([" << p0(0) << " " << p1(0) << " " << p2(0) << " " << p0(0) << "], [" << p0(1) << " " << p1(1) << " " << p2(1) << " " << p0(1) << "], 'r');";
        //o << "plot(" << mid_triangle(0) << ", " << mid_triangle(1) << ", 'ro'); pause;";
    }
    mid = 1.0/area*mid;
    
    //o << "plot(" << mid(0) << ", " << mid(1) << ", 'bo'); axis equal; pause";
    //o.eval();
}

bool table_tracking::center_contained(const geometry_msgs::Polygon& p, const Vector2d& c)
{
    int n = p.points.size();
    double is_signed;
    
    // check if c is on the same side of each line
    for (int i = 0; i < n; ++i) {
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

// check intersections of lines
bool table_tracking::are_overlapping(Vector2d& mida, const geometry_msgs::Polygon& a, Vector2d& midb, const geometry_msgs::Polygon& b)
{
    std::cout << "Got a new table, size of tables: " << tables.size() << std::endl;
    
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
    }*/
    
    if (center_contained(a, midb) || center_contained(b, mida)) {
        /*geometry_msgs::Polygon p;
        union_convex_hull(p, mida, a, midb, b);
        std::vector<double> px, py;
        px.resize(p.points.size());
        py.resize(p.points.size());
        for (int i = 0; i < p.points.size(); ++i) {
            px[i] = p.points[i].x;
            py[i] = p.points[i].y;
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
        return true;
    }
    
    return false;
}

void table_tracking::rectify_orientation(const Vector2d& c, geometry_msgs::Polygon& p)
{
    Vector2d p0(p.points[0].x, p.points[0].y);
    Vector2d p1(p.points[1].x, p.points[1].y);
    Vector2d v = p1 - p0;
    Vector2d o(-v(1), v(0));
    if (o.dot(c) < 0) {
        std::reverse(p.points.begin(), p.points.end());
    }
}

void table_tracking::union_convex_hull(geometry_msgs::Polygon& res, const Vector2d& mida, const geometry_msgs::Polygon& a, const Vector2d& midb, const geometry_msgs::Polygon& b)
{
    std::vector<Vector2d, aligned_allocator<Vector2d> > p;
    p.resize(a.points.size() + b.points.size());
    int n = a.points.size();
    for (int i = 0; i < n; ++i) {
        p[i] = Vector2d(a.points[i].x, a.points[i].y);
    }
    for (int i = 0; i < b.points.size(); ++i) {
        p[n + i] = Vector2d(b.points[i].x, b.points[i].y);
    }
    
    geometry_msgs::Point32 p32;
    std::vector<int> used;
    used.resize(p.size(), 0);

    bool reverse;
    int first_ind;
    int previous_ind;
    for (int i = 0; i < p.size(); ++i) {
        int ind = find_next_point(p[i], mida, p, used, reverse, true);
        if (ind != -1) {
            used[ind] = 1;
            used[i] = 1;
            if (reverse) {
                first_ind = ind;
                previous_ind = i;
            }
            else {
                first_ind = i;
                previous_ind = ind;
            }
            break;
        }
    }
    p32.x = p[first_ind](0);
    p32.y = p[first_ind](1);
    res.points.push_back(p32);
    while (previous_ind != first_ind) {
        p32.x = p[previous_ind](0);
        p32.y = p[previous_ind](1);
        res.points.push_back(p32);
        int ind = find_next_point(p[previous_ind], mida, p, used, reverse);
        previous_ind = ind;
        used[first_ind] = false;
    }
}

int table_tracking::find_next_point(const Vector2d& q, const Vector2d& c, std::vector<Vector2d, aligned_allocator<Vector2d> >& p, std::vector<int>& used, bool& reverse, bool first)
{
    Vector2d p0;
    Vector2d p1;
    Vector2d v;
    Vector2d o;
    bool found;
    for (int j = 0; j < p.size(); ++j) {
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
            if (!first) {
                // ERROR
            }
            p0 = p[j];
            p1 = q;
            v = -v;
            o = -o;
            reverse = true;
        }
        found = true;
        for (int k = 0; k < p.size(); ++k) {
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

bool table_tracking::add_detected_table(strands_perception_msgs::Table& table)
{
    std::vector<int> overlapping;
    Vector2d mid;
    compute_center_of_mass(mid, table.tabletop);
    rectify_orientation(mid, table.tabletop);
    for (int i = 0; i < tables.size(); ++i) {
        if (!are_overlapping(centers[i], tables[i]->tabletop, mid, table.tabletop)) {
            continue;
        }
        geometry_msgs::Polygon p;
        union_convex_hull(p, mid, table.tabletop, centers[i], tables[i]->tabletop);
        tables[i]->tabletop = p;
        compute_center_of_mass(mid, table.tabletop);
        centers[i] = mid;
        message_store.updateNamed(std::string("table") + tables[i]->table_id, *tables[i]);
        
        table = *tables[i];
        return false;
    }
    centers.push_back(mid);
    table.table_id = boost::lexical_cast<std::string>(tables.size());
    tables.push_back(boost::shared_ptr<strands_perception_msgs::Table>(new strands_perception_msgs::Table(table)));
    //Insert something with a name
    message_store.insertNamed(std::string("table") + table.table_id, table);
    return true;
}
