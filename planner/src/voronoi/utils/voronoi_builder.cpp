#include "planner/voronoi/utils/voronoi_builder.h"

void VoronoiBuilder::create_voronoi() {
  // prepares data structure for voronoi diagram
  std::vector<Point> points;
  std::vector<Segment> segments;

  // Iterates over exterior ring segments
  auto &exterior_ring = boost::geometry::exterior_ring(_map);
  for (std::size_t i = 0; i < exterior_ring.size() - 1; ++i) {
    const auto &vertex1 = exterior_ring[i];
    const auto &vertex2 = exterior_ring[i + 1];
    std::cout << "Exterior Segment: (" << boost::geometry::get<0>(vertex1)
              << ", " << boost::geometry::get<1>(vertex1) << ") - ("
              << boost::geometry::get<0>(vertex2) << ", "
              << boost::geometry::get<1>(vertex2) << ")\n";
    segments.push_back(Segment(
        boost::geometry::get<0>(vertex1), boost::geometry::get<1>(vertex1),
        boost::geometry::get<0>(vertex2), boost::geometry::get<1>(vertex2)));
  }

  // Iterate over interior rings segments
  auto &interior_rings = boost::geometry::interior_rings(_map);
  for (const auto &interior_ring : interior_rings) {
    for (std::size_t i = 0; i < interior_ring.size() - 1; ++i) {
      const auto &vertex1 = interior_ring[i];
      const auto &vertex2 = interior_ring[i + 1];
      std::cout << "Interior Segment: (" << boost::geometry::get<0>(vertex1)
                << ", " << boost::geometry::get<1>(vertex1) << ") - ("
                << boost::geometry::get<0>(vertex2) << ", "
                << boost::geometry::get<1>(vertex2) << ")\n";
      segments.push_back(Segment(
          boost::geometry::get<0>(vertex1), boost::geometry::get<1>(vertex1),
          boost::geometry::get<0>(vertex2), boost::geometry::get<1>(vertex2)));
    }
  }

  for (auto &segment : segments) {
    std::cout << "Segment: (" << segment.p0.a << ", " << segment.p0.b << ") - ("
              << segment.p1.a << ", " << segment.p1.b << ")\n";
  }
  std::cout << "Creating Voronoi diagram..." << std::endl;
  construct_voronoi(points.begin(), points.end(), segments.begin(),
                    segments.end(), &voronoi_diagram_);
  std::cout << "Voronoi diagram created!" << std::endl;
}

std::vector<std::pair<point_xy, point_xy>> VoronoiBuilder::get_voronoi_edges() {
  std::vector<std::pair<point_xy, point_xy>> voronoi_edges;
  for (auto edge = voronoi_diagram_.edges().begin();
       edge != voronoi_diagram_.edges().end(); ++edge) {
    if (!edge->is_finite() || is_edge_valid(edge)) {
      continue;
    }
    point_xy source =
        point_xy((double)edge->vertex0()->x(), (double)edge->vertex0()->y());
    point_xy target =
        point_xy((double)edge->vertex1()->x(), (double)edge->vertex1()->y());
    voronoi_edges.push_back(make_pair(source, target));
  }
  return voronoi_edges;
}

void VoronoiBuilder::compute_shortest_path() {
  std::vector<std::pair<point_xy, int>> point_index;
  int index = 0;
  // labelling vertexes
  for (auto vertex = voronoi_diagram_.vertices().begin();
       vertex != voronoi_diagram_.vertices().end(); ++vertex) {
    point_xy node(vertex->x(), vertex->y());
    point_index.push_back(make_pair(node, index));
  }
  std::vector<std::vector<pair<int, double>>> voronoi_graph;
  voronoi_graph.reserve(voronoi_diagram_.vertices().size() + 1);
  voronoi_graph.resize(voronoi_diagram_.vertices().size() + 1);
  // Get the source and target points of the current edge
  for (auto edge = voronoi_diagram_.edges().begin();
       edge != voronoi_diagram_.edges().end(); ++edge) {
    if (!edge->is_finite() || is_edge_valid(edge)) {
      continue;
    }
    point_xy source =
        point_xy((double)edge->vertex0()->x(), (double)edge->vertex0()->y());
    point_xy target =
        point_xy((double)edge->vertex1()->x(), (double)edge->vertex1()->y());
    // Calculate the Euclidean distance between the source and target points
    double weight =
        sqrt(pow(source.x() - target.x(), 2) + pow(source.y() - target.y(), 2));

    // Get the index of the source and target points in the Voronoi diagram
    int u = std::find_if(point_index.begin(), point_index.end(),
                         [&](const std::pair<point_xy, int> &p) {
                           return abs(p.first.x() - source.x()) < 0.0001 &&
                                  abs(p.first.y() - source.y()) < 0.0001;
                         })
                ->second;
    int v = std::find_if(point_index.begin(), point_index.end(),
                         [&](const std::pair<point_xy, int> &p) {
                           return abs(p.first.x() - target.x()) < 0.0001 &&
                                  abs(p.first.y() - target.y()) < 0.0001;
                         })
                ->second;

    voronoi_graph[u].push_back(make_pair(v, weight));
    voronoi_graph[v].push_back(make_pair(u, weight));
  }
  // Dijkstra's algorithm
  shortestPath(voronoi_graph, voronoi_graph.size(), 1);
}

void VoronoiBuilder::correct_geometry(
    boost::geometry::validity_failure_type &failure) {
  bool could_be_fixed = (failure == boost::geometry::failure_not_closed ||
                         failure == boost::geometry::failure_wrong_orientation);

  std::cout << "can boost::geometry::correct remedy invalidity? "
            << (could_be_fixed ? "possibly yes" : "no") << std::endl;
  if (could_be_fixed) {
    boost::geometry::correct(_map);
    std::cout << "after correction: "
              << (boost::geometry::is_valid(_map) ? "valid" : "still invalid")
              << std::endl;
    std::cout << "corrected geometry: " << boost::geometry::dsv(_map)
              << std::endl;
  } else {
    std::cout << "cannot be fixed" << std::endl;
  }
  return;
}

bool VoronoiBuilder::is_edge_valid(
    voronoi_diagram<double>::const_edge_iterator edge) {
  // check the edge ends in a outer poly's vertex
  for (const auto &point : _map.outer()) {
    if (edge->vertex0()->x() == point.x() &&
        edge->vertex0()->y() == point.y()) {
      return true;
    }
    if (edge->vertex1()->x() == point.x() &&
        edge->vertex1()->y() == point.y()) {
      return true;
    }
  }
  // check the edge ends in a inner polygon's vertex
  point_xy vertex0 = point_xy(edge->vertex0()->x(), edge->vertex0()->y());
  point_xy vertex1 = point_xy(edge->vertex1()->x(), edge->vertex1()->y());
  for (const auto &inner_ring : _map.inners()) {
    // Create a polygon object and assign the points to it.
    if (boost::geometry::within(vertex0, inner_ring) ||
        boost::geometry::within(vertex1, inner_ring)) {
      return true;
    }
    for (const auto &point : inner_ring) {
      if (vertex0.x() == point.x() && vertex0.y() == point.y()) {
        return true;
      }
      if (vertex1.x() == point.x() && vertex1.y() == point.y()) {
        return true;
      }
    }
  }
  return false;
}

// int main()
// {
//     // monitor execution time
//     auto start = high_resolution_clock::now();
//     VoronoiBuilder vb;
//     vb.create_voronoi_from_WTK(ament_index_cpp::get_package_share_directory("planner")
//     + "/data/polygon.txt"); auto stop = high_resolution_clock::now();

//     auto duration = duration_cast<microseconds>(stop - start);
//     std::cout << "Execution time: " << duration.count() << " microseconds.
//     Saving..." << std::endl;
//     vb.save_voronoi(ament_index_cpp::get_package_share_directory("planner")
//     +
//     "/data/boost_voronoi_edges.csv"); std::cout << "Done!" << std::endl;
//     return 0;
// }
