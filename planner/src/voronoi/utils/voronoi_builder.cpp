#include "planner/voronoi/utils/voronoi_builder.h"

void VoronoiBuilder::create_voronoi() {
  // prepares data structure for voronoi diagram
  std::vector<Point> points;
  std::vector<Segment> segments;

  // scales the polygon to avoid numerical issues
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
  // check if the vertexes are near map exterior ring
  for(auto &point : boost::geometry::exterior_ring(_map)){
    if (boost::geometry::distance(point, vertex0) < 0.05) {
      return false;
    }
    if (boost::geometry::distance(point, vertex1) < 0.05) {
      return false;
    }
  }

  return true;
}

