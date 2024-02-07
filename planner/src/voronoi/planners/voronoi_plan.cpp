#include "planner/voronoi/planners/voronoi_plan.hpp"

VoronoiPlan::VoronoiPlan(std::shared_ptr<MapInfo> &map_info)
    : _voronoi_builder(map_info->_map) {
  _map_info = map_info;
}

void VoronoiPlan::GenerateVoronoi(void) {
  _voronoi_builder.create_voronoi();

  std::cout << "Printing voronoi on file" << std::endl;
  std::ofstream file;
  string path = ament_index_cpp::get_package_share_directory("planner") +
                "/data/voronoi.txt";
  std::remove(path.c_str());
  file.open(path);
  // iteraint vertexes
  auto edges = _voronoi_builder.get_voronoi_edges();
  // connect each vertex to the start point -> create a set

  // creates set
  std::set<KDPoint> vertexes;
  for (auto &edge : edges) {
    std::cout << "Adding " << edge.first[0] << " " << edge.first[1]
              << std::endl;
    vertexes.insert(KDPoint{edge.first[0], edge.first[1]});
    vertexes.insert(KDPoint{edge.second[0], edge.second[1]});
  }

  // add connection from start point to each vertex
  for (auto &vertex : vertexes) {
    std::vector<KDPoint> segment = {vertex, _map_info->pt_start};
    if (_map_info->Collision(segment)) {
      continue;
    }
    edges.push_back(std::make_pair(_map_info->pt_start, vertex));
    edges.push_back(std::make_pair(_map_info->pt_end, vertex));
    for (auto v : _map_info->_victims) {
      std::vector<KDPoint> segment = {vertex, std::get<0>(v)};
      edges.push_back(std::make_pair(std::get<0>(v), vertex));
    }
  }

  for (auto &edge : edges) {
    file << edge.first[0] << " " << edge.first[1] << " " << edge.second[0]
         << " " << edge.second[1] << std::endl;
  }
  file.close();

  //   using namespace dijkstra;
  // shortestPath(voronoi_graph, voronoi_graph.size(), 1);
  // std::cout << "Dijkstra's algorithm done!" << std::endl;
}