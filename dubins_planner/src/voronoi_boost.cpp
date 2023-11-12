#include <cstdio>
#include <vector>
#include <iostream>
#include <boost/polygon/voronoi.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <chrono>


// I/O
#include <fstream>
#include <iostream>

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
string get_polygon_from_file(string filename)
{
    // Open the file
    std::ifstream file(filename);

    // Check if the file is open successfully
    if (!file.is_open())
    {
        std::cerr << "Error opening file: " << filename << std::endl;
        exit(1); // Return an error code
    }

    // Read the content of the file into a string
    std::string content((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());

    // Close the file
    file.close();

    return content;
}

// if the invalidity is only due to lack of closing points and/or wrongly oriented rings, then bg::correct can fix it
void correct_geometry(Polygon &polygon)
{
    boost::geometry::validity_failure_type failure;
    bool could_be_fixed = (failure == boost::geometry::failure_not_closed || boost::geometry::failure_wrong_orientation);

    std::cout << "can boost::geometry::correct remedy invalidity? " << (could_be_fixed ? "possibly yes" : "no") << std::endl;
    if (could_be_fixed)
    {
        boost::geometry::correct(polygon);
        std::cout << "after correction: " << (boost::geometry::is_valid(polygon) ? "valid" : "still invalid") << std::endl;
        std::cout << "corrected geometry: " << boost::geometry::dsv(polygon) << std::endl;
    }
    else
    {
        std::cout << "cannot be fixed" << std::endl;
        exit(1);
    }
}

bool is_edge_connected_to_polygon(voronoi_diagram<double>::const_edge_iterator edge, Polygon &polygon)
{
    // check the edge ends in a outer polygon's vertex
    for (const auto &point : polygon.outer())
    {
        if (edge->vertex0()->x() == point.x() && edge->vertex0()->y() == point.y())
        {
            return true;
        }
        if (edge->vertex1()->x() == point.x() && edge->vertex1()->y() == point.y())
        {
            return true;
        }
    }
    // check the edge ends in a inner polygon's vertex
    for (const auto &inner_ring : polygon.inners())
    {
        for (const auto &point : inner_ring)
        {
            if (edge->vertex0()->x() == point.x() && edge->vertex0()->y() == point.y())
            {
                return true;
            }
            if (edge->vertex1()->x() == point.x() && edge->vertex1()->y() == point.y())
            {
                return true;
            }
        }
    }

    return false;
}

void save_voronoi(const voronoi_diagram<double> &vd, Polygon &polygon )
{
    std::ofstream fout(ament_index_cpp::get_package_share_directory("dubins_planner") + "/data/boost_voronoi_edges.csv");
    fout << "x1,y1,x2,y2" << std::endl;
    for (voronoi_diagram<double>::const_edge_iterator edge = vd.edges().begin(); edge != vd.edges().end(); ++edge)
    {
        // std::cout << "Edge";
        // check if endpoints are finite
        if (edge->is_finite() && !is_edge_connected_to_polygon(edge, polygon))
        {
            std::cout << "Adding edge: " << edge->vertex0()->x() << ", " << edge->vertex0()->y() << " - " << edge->vertex1()->x() << ", " << edge->vertex1()->y() << std::endl;
            fout << edge->vertex0()->x() << "," << edge->vertex0()->y() << "," << edge->vertex1()->x() << "," << edge->vertex1()->y() << std::endl;
        }
    }
    return;
}

/// @brief ////////////////////////////////////////////////
/// @return ///////////////////////////////////////////////
int main()
{
    auto start = high_resolution_clock::now();

    std::string wkt_string = get_polygon_from_file(ament_index_cpp::get_package_share_directory("dubins_planner") + "/data/polygon.txt");
    std::cout << "WKT: " << wkt_string << "\n";
    boost::geometry::read_wkt(wkt_string, polygon);
    std::string reason;
    bool ok = boost::geometry::is_valid(polygon, reason);
    std::cout << "Expected: " << boost::geometry::dsv(polygon) << (ok ? "\nand got: VALID" : "\nand got: INVALID -> '" + reason + "'") << "\n";
    if (!ok)
    {
        correct_geometry(polygon);
    }

    // prepare data structure for voronoi diagram
    std::vector<Point> points;
    std::vector<Segment> segments;

    // Iterate over exterior ring segments
    auto &exterior_ring = boost::geometry::exterior_ring(polygon);
    for (std::size_t i = 0; i < exterior_ring.size() - 1; ++i)
    {
        const auto &vertex1 = exterior_ring[i];
        const auto &vertex2 = exterior_ring[i + 1];
        std::cout << "Exterior Segment: (" << boost::geometry::get<0>(vertex1) << ", " << boost::geometry::get<1>(vertex1) << ") - ("
                  << boost::geometry::get<0>(vertex2) << ", " << boost::geometry::get<1>(vertex2) << ")\n";
        segments.push_back(Segment(boost::geometry::get<0>(vertex1), boost::geometry::get<1>(vertex1), boost::geometry::get<0>(vertex2), boost::geometry::get<1>(vertex2)));
    }

    // Iterate over interior ring segments
    auto &interior_rings = boost::geometry::interior_rings(polygon);
    for (const auto &interior_ring : interior_rings)
    {
        for (std::size_t i = 0; i < interior_ring.size() - 1; ++i)
        {
            const auto &vertex1 = interior_ring[i];
            const auto &vertex2 = interior_ring[i + 1];
            std::cout << "Interior Segment: (" << boost::geometry::get<0>(vertex1) << ", " << boost::geometry::get<1>(vertex1) << ") - ("
                      << boost::geometry::get<0>(vertex2) << ", " << boost::geometry::get<1>(vertex2) << ")\n";
            segments.push_back(Segment(boost::geometry::get<0>(vertex1), boost::geometry::get<1>(vertex1), boost::geometry::get<0>(vertex2), boost::geometry::get<1>(vertex2)));
        }
    }

    for (auto &segment : segments)
    {
        std::cout << "Segment: (" << segment.p0.a << ", " << segment.p0.b << ") - ("
                  << segment.p1.a << ", " << segment.p1.b << ")\n";
    }

    voronoi_diagram<double> vd;
    construct_voronoi(points.begin(), points.end(), segments.begin(), segments.end(), &vd);
    save_voronoi(vd, polygon);

    auto stop = high_resolution_clock::now();

    auto duration = duration_cast<microseconds>(stop - start);
    std::cout << "Execution time: "<< duration.count() << " microseconds."<< std::endl;
}
