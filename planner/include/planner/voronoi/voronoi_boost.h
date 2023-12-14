#ifndef VORONOI_BOOST_H
#define VORONOI_BOOST_H
#include <cstdio>
#include <vector>
#include <iostream>
#include <chrono>

// I/O
#include <fstream>
#include <iostream>
#include <ament_index_cpp/get_package_share_directory.hpp>

// Boost
#include <boost/polygon/voronoi.hpp>
#include <boost/geometry.hpp>

using boost::polygon::high;
using boost::polygon::low;
using boost::polygon::voronoi_builder;
using boost::polygon::voronoi_diagram;
using boost::polygon::x;
using boost::polygon::y;
// measure execution time
using namespace std::chrono;
// print
using namespace std;

typedef boost::geometry::model::d2::point_xy<double> point;
typedef boost::geometry::model::polygon<point> Polygon;

Polygon polygon;

struct Point
{
    int a;
    int b;
    Point(int x, int y) : a(x), b(y) {}
};

struct Segment
{
    Point p0;
    Point p1;
    Segment(int x1, int y1, int x2, int y2) : p0(x1, y1), p1(x2, y2) {}
};
namespace boost
{
    namespace polygon
    {

        template <>
        struct geometry_concept<Point>
        {
            typedef point_concept type;
        };

        template <>
        struct point_traits<Point>
        {
            typedef int coordinate_type;

            static inline coordinate_type get(
                const Point &point, orientation_2d orient)
            {
                return (orient == HORIZONTAL) ? point.a : point.b;
            }
        };

        template <>
        struct geometry_concept<Segment>
        {
            typedef segment_concept type;
        };

        template <>
        struct segment_traits<Segment>
        {
            typedef int coordinate_type;
            typedef Point point_type;

            static inline point_type get(const Segment &segment, direction_1d dir)
            {
                return dir.to_int() ? segment.p1 : segment.p0;
            }
        };

    } // polygon
} // boost
class VoronoiBuilder
{
public:
    /// @brief data structure for the constructed voronoi diagram
    voronoi_diagram<double> voronoi_diagram_;

    /// @brief polygon data structure in which we load the WTK string
    Polygon polygon_;
    /// @brief create a polygon from a wkt string coming from a txt file
    /// @param filename name of the WTF .txt file to read

    void create_voronoi_from_WTK(string filename);
    /// @brief save the voronoi diagram into a csv file
    /// @param path


    void save_voronoi(string path);

private:
    // if the invalidity is only due to lack of closing points and/or wrongly oriented rings, then bg::correct can fix it
    /// @brief correct the geometry of a polygon
    /// @param polygon polygon to correct
    void correct_geometry(Polygon &polygon, boost::geometry::validity_failure_type &failure);

    bool is_edge_valid(voronoi_diagram<double>::const_edge_iterator edge, Polygon &polygon);
};
#endif