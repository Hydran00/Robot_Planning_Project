#include "planner/voronoi/utils/voronoi_builder.h"
#include "planner/map_info_node.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
class VoronoiPlan {
 private:
  VoronoiBuilder _voronoi_builder;
 public:
  // std::shared_ptr<MapInfo> &map_info;
  void GenerateVoronoi(void);
  std::tuple<std::vector<KDPoint>, double> run(void);
  VoronoiPlan(std::shared_ptr<MapInfo> &map_info);
};