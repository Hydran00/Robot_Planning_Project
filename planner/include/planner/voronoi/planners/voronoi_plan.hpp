#ifndef VORONOI_PLAN_HPP
#define VORONOI_PLAN_HPP
#include <boost/config.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/property_map/property_map.hpp>
#include <iostream>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "planner/map_info_node.hpp"
#include "planner/voronoi/utils/voronoi_builder.h"
#include "planner/voronoi/utils/combinations.h"
#include "planner/dubins/dubins.h"

class VoronoiPlan {
 private:
  VoronoiBuilder _voronoi_builder;

 public:
  std::shared_ptr<MapInfo> _map_info;
  std::pair<std::vector<KDPoint>, double> GetPlan(void);
  VoronoiPlan(std::shared_ptr<MapInfo> &map_info);
  // radius in which we connect landmarks (start,end,victims) to the voronoi
  const double conn_radius = 5.0;
};
#endif  // VORONOI_PLAN_HPP