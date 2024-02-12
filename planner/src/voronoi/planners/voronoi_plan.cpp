#include "planner/voronoi/planners/voronoi_plan.hpp"

VoronoiPlan::VoronoiPlan(std::shared_ptr<MapInfo> &map_info)
    : _voronoi_builder(map_info->_map) {
  _map_info = map_info;
}

std::pair<std::vector<KDPoint>, double> VoronoiPlan::GetPlan(void) {
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
  // show voronoi on the map
  _map_info->set_voronoi(edges);

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
  std::cout << "A"<< std::endl;
  // labelling vertices
  std::vector<std::pair<KDPoint, int>> point_index;
  int index = 0;
  // labelling vertices
  for (auto vertex : vertices) {
    point_index.push_back(make_pair(vertex, index));
    index++;
  }
  std::cout << "B"<< std::endl;

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
  std::cout << "C"<< std::endl;

  std::cout << "C2"<< std::endl;
  Dijkstra dijkstra(edge_array);
  // gets start vertex index
  std::cout << "C3"<< std::endl;
  
  int start =
      std::find_if(point_index.begin(), point_index.end(),
                   [&](const std::pair<KDPoint, int> &p) {
                     return distance(p.first, _map_info->pt_start) < 1.0e-6;
                   })
          ->second;
  ////////////////////////////////////////////////////////////////////////////
  /////////////////// EXPLORATION METHOD/// //////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////

  // generates all possible combinations of the victims (FACTORIAL TIME
  // COMPLEXITY)
  std::cout << "C3"<<std::endl;

  typedef std::tuple<KDPoint, double> Victim;
  typedef std::vector<Victim> victim_combination;
  std::vector<victim_combination> all_combinations;
  std::vector<int> indices(_map_info->_victims.size());
  std::cout << "C4"<<std::endl;
  std::iota(indices.begin(), indices.end(), 0);
   std::vector<std::vector<int>> combinations = all_permutations_with_subsets(indices);
  // auto combinations = all_permutations_with_subsets(indices);
  for (auto &combination : combinations) {
    victim_combination vc;
    for (auto &i : combination) {
      vc.push_back(_map_info->_victims[i]);
      std::cout <<"C5"<<std::endl;
    }
      std::cout <<"C6"<<std::endl;

    all_combinations.push_back(vc);
  }
  std::cout << "D"<< std::endl;

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
      // no path found
      if (p.size() == 0) {
        cost = std::numeric_limits<double>::max();
        break;
      }
      // insert path into the vector excluding the last vertex
      for (size_t i = 0; i < p.size() - 1; i++) {
        path.push_back(point_index[p[i]].first);
      }
      // cost considers also victims value
      cost += dijkstra.kD[start] - std::get<1>(v);
      current_vertex = end;
    }
    // inserts path to gate
    int end =
        std::find_if(point_index.begin(), point_index.end(),
                     [&](const std::pair<KDPoint, int> &p) {
                       return distance(p.first, _map_info->pt_end) < 1.0e-6;
                     })
            ->second;
    std::vector<int> p = dijkstra.get_shortest_path(current_vertex, end);
    if (p.size() == 0) {
      cost += std::numeric_limits<double>::max();
      break;
    }
    for (size_t i = 0; i < p.size(); i++) {
      path.push_back(point_index[p[i]].first);
    }

    paths.push_back(std::make_pair(path, cost));
  }

  auto best_path =
      std::min_element(paths.begin(), paths.end(),
                       [](const std::pair<std::vector<KDPoint>, double> &p1,
                          const std::pair<std::vector<KDPoint>, double> &p2) {
                         return p1.second < p2.second;
                       });

  std::ofstream file2;
  string path2 = ament_index_cpp::get_package_share_directory("planner") +
                 "/data/best_path_voronoi.txt";
  std::remove(path2.c_str());
  file2.open(path2);
  for (auto &p : best_path->first) {
    file2 << p[0] << " " << p[1] << std::endl;
  }
  file2.close();
  return *best_path;
}