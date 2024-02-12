
#include "planner/voronoi/utils/dijkstra.h"
Dijkstra::Dijkstra(std::vector<VEdge> edges) {
  using namespace boost::adaptors;
  size_t max_node = 0;
  boost::partial_sort_copy(edges | transformed([](VEdge const &e) -> size_t {
                             return std::max(e.source, e.target);
                           }),
                           boost::make_iterator_range(&max_node, &max_node + 1),
                           std::greater<size_t>());
  auto e = edges | transformed([](VEdge const &ve) {
             return std::make_pair(ve.source, ve.target);
           });
  kGraph = graph_t(e.begin(), e.end(), edges.begin(), max_node + 1);
}

std::pair<std::vector<int>,double> Dijkstra::get_shortest_path(int start, int end) {
  weight_map_t kWeightMap = boost::get(&VEdge::weight, kGraph);

  vertex_descriptor kS = vertex(start, kGraph);
  std::vector<vertex_descriptor> kP = std::vector<vertex_descriptor>(num_vertices(kGraph)+1);
  std::vector<double> kD = std::vector<double>(num_vertices(kGraph)+1);
  auto id = get(boost::vertex_index, kGraph);
  dijkstra_shortest_paths(
      kGraph, kS,
      distance_map(boost::make_iterator_property_map(kD.begin(), id))
          .predecessor_map(boost::make_iterator_property_map(kP.begin(), id))
          .weight_map(kWeightMap));

  int current_vertex = end;
  // if no path found
  if ((int)kP[current_vertex] == current_vertex) {
    return std::make_pair(std::vector<int>(), std::numeric_limits<double>::max());
  }
  // return path from start to end
  std::vector<int> path;
  while (current_vertex != start) {
    path.push_back(current_vertex);
    // std::cout << "AAAADistance from " << current_vertex << " to " <<start<<" is " <<kD[current_vertex] << std::endl;
    current_vertex = kP[current_vertex];
  }
  path.push_back(start);
  std::reverse(path.begin(), path.end());
  return std::make_pair(path, kD[end]);
}
