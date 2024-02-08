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
  // iteraint vertices
  auto edges = _voronoi_builder.get_voronoi_edges();

  // creates set for extracting unique vertices
  std::set<KDPoint> vertices;

  // add all vertices to the set
  for (auto &edge : edges) {
    vertices.insert(KDPoint{edge.first[0], edge.first[1]});
    vertices.insert(KDPoint{edge.second[0], edge.second[1]});
  }

  auto distance = [](KDPoint a, KDPoint b) {
    return std::sqrt(std::pow(a[0] - b[0], 2) + std::pow(a[1] - b[1], 2));
  };

  // increment voronoi connections
  std::vector<KDPoint> segment = {{0.0, 0.0}, {0.0, 0.0}};
  for (auto vertex : vertices) {
    segment[0] = vertex;
    // add connection from start point to each vertex
    segment[1] = _map_info->pt_start;
    if (!_map_info->Collision(segment) &&
        distance(vertex, _map_info->pt_start) < conn_radius) {
      edges.push_back(std::make_pair(_map_info->pt_start, vertex));
    }
    // add connection from end point to each vertex
    segment[1] = _map_info->pt_end;
    if (!_map_info->Collision(segment) &&
        distance(vertex, _map_info->pt_end) < conn_radius) {
      edges.push_back(std::make_pair(_map_info->pt_end, vertex));
    }
    // add connection from victims to each vertex
    for (auto v : _map_info->_victims) {
      segment[1] = std::get<0>(v);
      if (!_map_info->Collision(segment) &&
          distance(vertex, std::get<0>(v)) < conn_radius) {
        edges.push_back(std::make_pair(std::get<0>(v), vertex));
      }
    }
  }
  // add connection from start point to end point
  segment[0] = _map_info->pt_start;
  segment[1] = _map_info->pt_end;
  if (!_map_info->Collision(segment)) {
    edges.push_back(std::make_pair(_map_info->pt_start, _map_info->pt_end));
  }
  // add connection from start point to each victim
  for (auto v : _map_info->_victims) {
    segment[1] = std::get<0>(v);
    if (!_map_info->Collision(segment)) {
      edges.push_back(std::make_pair(_map_info->pt_start, std::get<0>(v)));
    }
  }
  // add connection from end point to each victim
  segment[0] = _map_info->pt_end;
  for (auto v : _map_info->_victims) {
    segment[1] = std::get<0>(v);
    if (!_map_info->Collision(segment)) {
      edges.push_back(std::make_pair(_map_info->pt_end, std::get<0>(v)));
    }
  }
  // add connection from each victim to each victim
  for (size_t i = 0; i < _map_info->_victims.size() - 1; i++) {
    for (size_t j = i + 1; j < _map_info->_victims.size(); j++) {
      segment[0] = std::get<0>(_map_info->_victims[i]);
      segment[1] = std::get<0>(_map_info->_victims[j]);
      if (!_map_info->Collision(segment)) {
        edges.push_back(std::make_pair(std::get<0>(_map_info->_victims[i]),
                                       std::get<0>(_map_info->_victims[j])));
      }
    }
  }
  for (auto &edge : edges) {
    file << edge.first[0] << " " << edge.first[1] << " " << edge.second[0]
         << " " << edge.second[1] << std::endl;
  }
  file.close();

  // adding start, end and victims to the vertices
  vertices.insert(_map_info->pt_start);
  vertices.insert(_map_info->pt_end);
  for (auto v : _map_info->_victims) {
    vertices.insert(std::get<0>(v));
  }

  // labelling vertices
  std::vector<std::pair<KDPoint, int>> point_index;
  int index = 0;
  // labelling vertices
  for (auto vertex : vertices) {
    point_index.push_back(make_pair(vertex, index));
    index++;
  }

  std::vector<std::vector<pair<int, double>>> final_voronoi;
  final_voronoi.reserve(vertices.size() + 1);
  final_voronoi.resize(vertices.size() + 1);

  using namespace boost;
  typedef adjacency_list<listS, vecS, directedS, no_property,
                         property<edge_weight_t, int>>
      graph_t;
  typedef graph_traits<graph_t>::vertex_descriptor vertex_descriptor;
  // create edge array
  std::vector<VEdge> edge_array;
  // creaate weights array
  std::vector<double> weights;
  weights.resize(final_voronoi.size());
  // create weighted graph
  for (auto edge : edges) {
    int u = std::find_if(point_index.begin(), point_index.end(),
                         [&](const std::pair<KDPoint, int> &p) {
                           return distance(p.first, edge.first) < 1.0e-6;
                         })
                ->second;
    int v = std::find_if(point_index.begin(), point_index.end(),
                         [&](const std::pair<KDPoint, int> &p) {
                           return distance(p.first, edge.second) < 1.0e-6;
                         })
                ->second;
    VEdge e{u, v, distance(edge.first, edge.second)};
    edge_array.push_back(e);
  }
  Dijkstra dijkstra(edge_array);
  // get start vertex index 
  int start = std::find_if(point_index.begin(), point_index.end(),
                           [&](const std::pair<KDPoint, int> &p) {
                             return distance(p.first, _map_info->pt_start) < 1.0e-6;
                           })
                  ->second;
  int end = std::find_if(point_index.begin(), point_index.end(),
                         [&](const std::pair<KDPoint, int> &p) {
                           return distance(p.first, _map_info->pt_end) < 1.0e-6;
                         })
              ->second;
  std::cout << "Start is "<< start << " and end is " << end << std::endl;
  std::vector<int> shortest_path = dijkstra.get_shortest_path(start,end);
  // print path
  for (auto p : shortest_path) {
    std::cout << p << " "<< std::endl;
  }
}