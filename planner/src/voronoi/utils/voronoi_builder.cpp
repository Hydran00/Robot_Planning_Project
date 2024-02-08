#include "planner/voronoi/utils/voronoi_builder.h"

void VoronoiBuilder::create_voronoi() {
  // prepares data structure for voronoi diagram
  std::vector<Point> points;
  std::vector<Segment> segments;

  // scales the polygon (1000) to avoid numerical issues
  polygon _scaled_map;
  // Apply scale transformation
  boost::geometry::strategy::transform::scale_transformer<double, 2, 2> scale(
      scale_factor);
  boost::geometry::transform(_map, _scaled_map, scale);

  // Iterates over exterior ring segments
  auto &exterior_ring = boost::geometry::exterior_ring(_scaled_map);
  for (std::size_t i = 0; i < exterior_ring.size() - 1; ++i) {
    const auto &vertex1 = exterior_ring[i];
    const auto &vertex2 = exterior_ring[i + 1];
    segments.push_back(Segment(
        boost::geometry::get<0>(vertex1), boost::geometry::get<1>(vertex1),
        boost::geometry::get<0>(vertex2), boost::geometry::get<1>(vertex2)));
  }

  // Iterate over interior rings segments
  auto &interior_rings = boost::geometry::interior_rings(_scaled_map);
  for (const auto &interior_ring : interior_rings) {
    for (std::size_t i = 0; i < interior_ring.size() - 1; ++i) {
      const auto &vertex1 = interior_ring[i];
      const auto &vertex2 = interior_ring[i + 1];
      segments.push_back(Segment(
          boost::geometry::get<0>(vertex1), boost::geometry::get<1>(vertex1),
          boost::geometry::get<0>(vertex2), boost::geometry::get<1>(vertex2)));
    }
  }
  std::cout << "Creating Voronoi diagram..." << std::endl;
  construct_voronoi(points.begin(), points.end(), segments.begin(),
                    segments.end(), &voronoi_diagram_);
  std::cout << "Voronoi diagram created!" << std::endl;
  // _map = map_copy;
}

std::vector<std::pair<KDPoint, KDPoint>> VoronoiBuilder::get_voronoi_edges() {
  std::vector<std::pair<KDPoint, KDPoint>> voronoi_edges;
  for (auto edge = voronoi_diagram_.edges().begin();
       edge != voronoi_diagram_.edges().end(); ++edge) {
    if (edge->is_finite() && is_edge_valid(edge)) {
      KDPoint source = {edge->vertex0()->x() / scale_factor,
                        edge->vertex0()->y() / scale_factor};
      KDPoint target = {edge->vertex1()->x() / scale_factor,
                        edge->vertex1()->y() / scale_factor};
      voronoi_edges.push_back(make_pair(source, target));
    }
  }
  return voronoi_edges;
  // return
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
  // check if the edge ends in the map
  point_xy vertex0 = point_xy(edge->vertex0()->x() / scale_factor,
                              edge->vertex0()->y() / scale_factor);
  point_xy vertex1 = point_xy(edge->vertex1()->x() / scale_factor,
                              edge->vertex1()->y() / scale_factor);
  Linestring l;
  boost::geometry::append(l, vertex0);
  boost::geometry::append(l, vertex1);
  if (!boost::geometry::within(l,_map)) {
    return false;
  }
  // check if the edge ends in a outer poly's vertex
  // for (const auto &point : _map.outer()) {
  //   if (abs(edge->vertex0()->x() / scale_factor - point.x() < 0.1) &&
  //       abs(edge->vertex0()->y() / scale_factor - point.y() < 0.1)) {
  //     std::cout << "A" << std::endl;
  //     return false;
  //   }
  //   if (abs(edge->vertex1()->x() / scale_factor - point.x() < 0.1) &&
  //       abs(edge->vertex1()->y() / scale_factor - point.y() < 0.1)) {
  //     std::cout << "B" << std::endl;
  //     return false;
  //   }
  // }
  // check if the edge ends in a inner polygon's vertex
  // for (const auto &inner_ring : _scaled_map.inners()) {
  //   for (const auto &point : inner_ring) {
  //     if (abs(vertex0.x() / scale_factor - point.x()) < 0.2 &&
  //         abs(vertex0.y() / scale_factor - point.y()) < 0.2) {
  //       std::cout << "D" << std::endl;

  //       return false;
  //     }
  //     if (abs(vertex1.x() / scale_factor - point.x()) < 0.2 &&
  //         abs(vertex1.y() / scale_factor - point.y()) < 0.2) {
  //       std::cout << "E" << std::endl;

  //       return false;
  //     }
  //   }
  // }
  return true;
}

