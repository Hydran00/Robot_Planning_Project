#ifndef VORONOI_BOOST_H
#define VORONOI_BOOST_H
#include <chrono>
#include <cstdio>
#include <iostream>
#include <vector>

// I/O
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fstream>
#include <iostream>

// Boost
#include <boost/geometry.hpp>
#include <boost/polygon/voronoi.hpp>
#include <boost/geometry/algorithms/transform.hpp>
#include "planner/voronoi/utils/dijkstra.h"
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

typedef boost::geometry::model::d2::point_xy<double> point_xy;
typedef boost::geometry::model::polygon<point_xy> polygon;
// segment

// polygon polygon;

struct Point {
  int a;
  int b;
  Point(int x, int y) : a(x), b(y) {}
};

struct Segment {
  Point p0;
  Point p1;
  Segment(int x1, int y1, int x2, int y2) : p0(x1, y1), p1(x2, y2) {}
};
namespace boost {
namespace polygon {

template <>
struct geometry_concept<Point> {
  typedef point_concept type;
};

template <>
struct point_traits<Point> {
  typedef int coordinate_type;

  static inline coordinate_type get(const Point &point, orientation_2d orient) {
    return (orient == HORIZONTAL) ? point.a : point.b;
  }
};

template <>
struct geometry_concept<Segment> {
  typedef segment_concept type;
};

template <>
struct segment_traits<Segment> {
  typedef int coordinate_type;
  typedef Point point_type;

  static inline point_type get(const Segment &segment, direction_1d dir) {
    return dir.to_int() ? segment.p1 : segment.p0;
  }
};

}  // namespace polygon
}  // namespace boost
class VoronoiBuilder {
 public:
  /// @brief data structure for the constructed voronoi diagram
  voronoi_diagram<double> voronoi_diagram_;
  const double scale_factor = 10.0;
  polygon _map;
  void create_voronoi();
  std::vector<std::pair<point_xy, point_xy>> get_voronoi_edges();
  void compute_shortest_path();
  VoronoiBuilder(polygon &map) : _map(map) {}
 private:
  void correct_geometry(boost::geometry::validity_failure_type &failure);
  bool is_edge_valid(voronoi_diagram<double>::const_edge_iterator edge);
};
#endif