#include "planner/voronoi/planners/voronoi_plan.hpp"

VoronoiPlan::VoronoiPlan(std::shared_ptr<MapInfo> &map_info)
    : _voronoi_builder(map_info->_map) {
  _map_info = map_info;
}

void VoronoiPlan::GenerateVoronoi(void) {
  _voronoi_builder.create_voronoi();

  std::ofstream file;
  string file_path = ament_index_cpp::get_package_share_directory("planner") +
                     "/data/voronoi.txt";
  std::remove(file_path.c_str());
  file.open(file_path);
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

  ////////////////////////////////////////////////////////////////////////////
  /////////////////// VORONOI AUGMENTATION ///////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////

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
  // add connection between victims
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
  std::cout << "Printing voronoi on file" << std::endl;
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

  // create edge array
  std::vector<VEdge> edge_array;
  // creaate weights array
  std::vector<double> weights;
  weights.resize(vertices.size() + 1);
  // create weighted graph
  for (auto edge : edges) {
    int u = std::find_if(point_index.begin(), point_index.end(),
                         [&](const std::pair<KDPoint, int> &p) {
                           return distance(p.first, edge.first) < 1.0e-2;
                         })
                ->second;
    int v = std::find_if(point_index.begin(), point_index.end(),
                         [&](const std::pair<KDPoint, int> &p) {
                           return distance(p.first, edge.second) < 1.0e-2;
                         })
                ->second;
    if (u == v) {
      continue;
    }

    VEdge e1{u, v, distance(edge.first, edge.second)};
    VEdge e2{v, u, distance(edge.first, edge.second)};
    edge_array.push_back(e1);
    edge_array.push_back(e2);
  }
  Dijkstra dijkstra(edge_array);
  // get start vertex index
  int start =
      std::find_if(point_index.begin(), point_index.end(),
                   [&](const std::pair<KDPoint, int> &p) {
                     return distance(p.first, _map_info->pt_start) < 1.0e-6;
                   })
          ->second;
  ////////////////////////////////////////////////////////////////////////////
  /////////////////// EXPLORATION METHOD/// //////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////

  // generates all possible combinations of the victims
  typedef std::tuple<KDPoint, double> Victim;
  typedef std::vector<Victim> victim_combination;
  std::vector<victim_combination> all_combinations;
  std::vector<int> indices(_map_info->_victims.size());
  std::iota(indices.begin(), indices.end(), 0);
  std::vector<std::vector<int>> combinations = generateAllCombinations(indices);
  for (auto &combination : combinations) {
    victim_combination vc;
    for (auto &i : combination) {
      vc.push_back(_map_info->_victims[i]);
    }
    all_combinations.push_back(vc);
  }
  // for each combination of victims finds the shortest path
  std::vector<std::pair<std::vector<KDPoint>, double>> paths;
  for (auto &combination : all_combinations) {
    std::vector<KDPoint> path;
    double cost = 0;
    int current_vertex = start;
    for (auto &v : combination) {
      int end =
          std::find_if(point_index.begin(), point_index.end(),
                       [&](const std::pair<KDPoint, int> &p) {
                         return distance(p.first, std::get<0>(v)) < 1.0e-6;
                       })
              ->second;
      std::vector<int> p = dijkstra.get_shortest_path(current_vertex, end);
      if (p.size() == 0) {
        std::cout << "No path found" << std::endl;
        cost = std::numeric_limits<double>::max();
        break;
      }
      // path.push_back(_map_info->pt_start);
      // for (auto &i : p) {
      //   path.push_back(point_index[i].first);
      // }
      path.push_back(_map_info->pt_start);
      for (size_t i = 1; i < p.size(); i++) {
        path.push_back(point_index[p[i]].first);
      }
      // cost consider also victims value
      cost += dijkstra.kD[start] - std::get<1>(v);
      current_vertex = end;
      std::cout << "KDstart " << dijkstra.kD[start] << " KDend "
                << dijkstra.kD[end] << std::endl;
      std::cout << "Saving victim " << std::get<0>(v)[0] << " "
                << std::get<0>(v)[1] << " with cost " << cost << std::endl;
    }
    path.push_back(_map_info->pt_end);

    std::cout << "Cost is " << cost << std::endl;
    paths.push_back(std::make_pair(path, cost));
    std::cout << "Pushing " << cost << ", path size is " << path.size()
              << std::endl;
    std::cout << "//////////////////" << std::endl;
  }

  auto best_path =
      std::min_element(paths.begin(), paths.end(),
                       [](const std::pair<std::vector<KDPoint>, double> &p1,
                          const std::pair<std::vector<KDPoint>, double> &p2) {
                         return p1.second < p2.second;
                       });
  std::cout << "Best path is: "
            << "size:" << best_path->first.size()
            << " cost: " << best_path->second << std::endl;
  for (auto &p : best_path->first) {
    std::cout << p[0] << " " << p[1] << " " << std::endl;
  }
  std::ofstream file2;
  string path2 = ament_index_cpp::get_package_share_directory("planner") +
                 "/data/best_path_voronoi.txt";
  std::remove(path2.c_str());
  file2.open(path2);
  for (auto &p : best_path->first) {
    file2 << p[0] << " " << p[1] << std::endl;
  }
}