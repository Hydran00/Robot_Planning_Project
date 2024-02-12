#ifndef __DIJKSTRA_H__
#define __DIJKSTRA_H__
#include <boost/config.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/property_map/property_map.hpp>
#include <boost/range/adaptors.hpp>
#include <boost/range/algorithm.hpp>
#include <fstream>
#include <iostream>
struct VEdge {
  int source, target;
  double weight;
  // custom variables here
};
class Dijkstra {
  using graph_t =
      boost::adjacency_list<boost::listS, boost::vecS, boost::directedS,
                            boost::no_property, VEdge>;
  using vertex_descriptor = boost::graph_traits<graph_t>::vertex_descriptor;
  using edge_descriptor = boost::graph_traits<graph_t>::edge_descriptor;
  using weight_map_t = boost::property_map<graph_t, double VEdge::*>::type;

 public:
  Dijkstra(std::vector<VEdge>);
  ~Dijkstra() {}

  std::pair<std::vector<int>, double> get_shortest_path(int start, int end);

  void print_path();
  void generate_dot_file();

 private:
  graph_t kGraph;
};

using namespace std;
// typedef pair<int, double> Node;
// void shortestPath(const vector<vector<pair<int, double>>> &adj, int V, int
// src);
#endif  // DIJKSTRA_H