
#include "planner/voronoi/utils/dijkstra.h"
Dijkstra::Dijkstra(std::vector<VEdge> edges) {
  using namespace boost::adaptors;
  size_t max_node;
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

std::vector<int> Dijkstra::get_shortest_path(int start, int end) {
  weight_map_t kWeightMap = boost::get(&VEdge::weight, kGraph);
  vertex_descriptor kS = vertex(start, kGraph);
  kP = std::vector<vertex_descriptor>(num_vertices(kGraph));
  kD = std::vector<double>(num_vertices(kGraph));

  dijkstra_shortest_paths(
      kGraph, kS,
      predecessor_map(boost::make_iterator_property_map(
                          kP, get(boost::vertex_index, kGraph)))
          .distance_map(boost::make_iterator_property_map(
              kS, get(boost::vertex_index, kGraph)))
          .weight_map(kWeightMap));
  // return path from start to end
  std::vector<int> path;
  int current_vertex = end;
  while (current_vertex != start) {
    // std::cout << "current vertex is " << current_vertex << std::endl;
    path.push_back(current_vertex);
    current_vertex = kP[current_vertex];
  }
  path.push_back(start);
  std::reverse(path.begin(), path.end());
  return path;
}

void Dijkstra::print_path() {
  std::cout << "distances and parents:" << std::endl;
  boost::graph_traits<graph_t>::vertex_iterator vi, vend;

  for (boost::tie(vi, vend) = vertices(kGraph); vi != vend; ++vi) {
    std::cout << "distance(" << *vi << ") = " << kD[*vi] << ", ";
    std::cout << "parent(" << *vi << ") = " << kP[*vi] << "\n";
  }
}
