#ifndef VORONOI_PLAN_HPP
#define VORONOI_PLAN_HPP
#include <boost/config.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/property_map/property_map.hpp>
#include <iostream>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "planner/dubins/dubins.h"
#include "planner/map_info_node.hpp"
#include "planner/motionplanning.hpp"
#include "planner/voronoi/utils/permutations.h"
#include "planner/voronoi/utils/voronoi_builder.h"

class VoronoiPlan : MotionPlanning {
 private:
  VoronoiBuilder _voronoi_builder;

 public:
  std::vector<KDPoint> OptimisePath(std::vector<KDPoint> &path);
  VoronoiPlan(std::shared_ptr<MapInfo> &map_info);
  std::tuple<std::vector<KDPoint>, double> run(void);
  // radius in which we connect landmarks (start,end,victims) to the voronoi
  const double conn_radius = 5.0;
};
#endif  // VORONOI_PLAN_HPP