#include "planner/voronoi/planners/voronoi_plan.hpp"

VoronoiPlan::VoronoiPlan(std::shared_ptr<MapInfo> &map_info)
    : _voronoi_builder(map_info->_map) {}

void VoronoiPlan::GenerateVoronoi(void) {
  _voronoi_builder.create_voronoi();

  std::cout << "Printing voronoi on file" << std::endl;
  std::ofstream file;
  string path = ament_index_cpp::get_package_share_directory("planner") +
                "/data/voronoi.txt";
  std::remove(path.c_str());
  file.open(path);
  // iteraint vertexes
  for (auto &edge : _voronoi_builder.get_voronoi_edges()) {
    file << edge.first.x() << " " << edge.first.y() << " " << edge.second.x()
         << " " << edge.second.y() << std::endl;
  }
  file.close();

  //   using namespace dijkstra;
  // shortestPath(voronoi_graph, voronoi_graph.size(), 1);
  // std::cout << "Dijkstra's algorithm done!" << std::endl;
}