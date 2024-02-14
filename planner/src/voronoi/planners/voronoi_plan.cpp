#include "planner/voronoi/planners/voronoi_plan.hpp"
VoronoiPlan::VoronoiPlan(std::shared_ptr<MapInfo> &map_info)
    : MotionPlanning(map_info), _voronoi_builder(map_info->_map) {}

// RRTPlan::RRTPlan(std::shared_ptr<MapInfo> &map_info)
//     : MotionPlanning(map_info), _rrt(map_info->_victims, this->seed) {
//   _display = map_info->_show_graphics;
//   // call rrt constructor
//   _rrt.set_root(MotionPlanning::_pt_start);
// }


std::tuple<std::vector<KDPoint>, double> VoronoiPlan::run(void) {
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
  // gets start vertex index

  int start =
      std::find_if(point_index.begin(), point_index.end(),
                   [&](const std::pair<KDPoint, int> &p) {
                     return distance(p.first, _map_info->pt_start) < 1.0e-6;
                   })
          ->second;
  int gate =
      std::find_if(point_index.begin(), point_index.end(),
                   [&](const std::pair<KDPoint, int> &p) {
                     return distance(p.first, _map_info->pt_end) < 1.0e-6;
                   })
          ->second;
  ////////////////////////////////////////////////////////////////////////////
  /////////////////// EXPLORATION METHOD/// //////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////

  // generates all possible combinations of the victims (FACTORIAL TIME
  // COMPLEXITY)
  typedef std::tuple<KDPoint, double> Victim;
  typedef std::vector<Victim> victim_combination;
  std::vector<victim_combination> all_combinations;
  std::vector<int> indices(_map_info->_victims.size());
  std::iota(indices.begin(), indices.end(), 0);
  std::vector<std::vector<int>> combinations;

  auto victims_list = _map_info->_victims;

  if (_map_info->_exploration_method == "brute_force") {
    combinations = all_permutations_with_subsets(indices);
  } else {
    // heuristic
    std::cout << "USING HEURISTIC" << std::endl;
    combinations = all_combinations_with_subsets(indices);
    // sorting in descending order since the sign is inverted later
    std::sort(victims_list.begin(), victims_list.end(), [](Victim a, Victim b) {
      return std::get<1>(a) > std::get<1>(b);
    });
    for (auto v : victims_list) {
      std::cout << "cost: " << -std::get<1>(v) << std::endl;
    }
  }

  // retrieves the victims combination starting from indices
  for (auto &combination : combinations) {
    victim_combination vc;
    for (auto &i : combination) {
      vc.push_back(victims_list[i]);
    }

    all_combinations.push_back(vc);
  }

  // for each combination of victims finds the shortest path
  std::vector<std::tuple<std::vector<KDPoint>, double>> paths;
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
      std::pair<std::vector<int>, double> path_cost =
          dijkstra.get_shortest_path(current_vertex, end);
      // no path found
      if (path_cost.first.size() == 0) {
        cost = std::numeric_limits<double>::max();
        break;
      }
      // insert path into the vector excluding the last vertex
      for (size_t i = 0; i < path_cost.first.size() - 1; i++) {
        path.push_back(point_index[path_cost.first[i]].first);
      }
      // cost travel distance and victims value
      cost += path_cost.second - std::get<1>(v);
      current_vertex = end;
    }
    // inserts path to gate
    std::pair<std::vector<int>, double> last_path_cost =
        dijkstra.get_shortest_path(current_vertex, gate);
    if (last_path_cost.first.size() == 0) {
      cost += std::numeric_limits<double>::max();
      break;
    }
    for (size_t i = 0; i < last_path_cost.first.size(); i++) {
      path.push_back(point_index[last_path_cost.first[i]].first);
    }
    cost += last_path_cost.second;
    paths.push_back(std::make_tuple(path, cost));
  }

  std::cout << "START IS " << start << std::endl;
  // check if every path's cost is infinite
  if (std::all_of(paths.begin(), paths.end(),
                  [](std::tuple<std::vector<KDPoint>, double> p) {
                    return std::get<1>(p) == std::numeric_limits<double>::max();
                  })) {
    std::cout << "NO PATH FOUND" << std::endl;
    return std::make_pair(std::vector<KDPoint>(),
                          std::numeric_limits<double>::max());
  }
  auto best_path_it =
      std::min_element(paths.begin(), paths.end(),
                       [](const std::tuple<std::vector<KDPoint>, double> &p1,
                          const std::tuple<std::vector<KDPoint>, double> &p2) {
                         return std::get<1>(p1) < std::get<1>(p2);
                       });
  // print total cost
  std::cout << "TOTAL COST: " << std::get<1>(*best_path_it) << std::endl;
  std::ofstream file2;
  string path2 = ament_index_cpp::get_package_share_directory("planner") +
                 "/data/voronoi_path.txt";
  std::remove(path2.c_str());

  file2.open(path2);
  for (auto &p : std::get<0>(*best_path_it)) {
    file2 << p[0] << " " << p[1] << std::endl;
  }
  file2.close();

  return *best_path_it;
}

std::vector<KDPoint> VoronoiPlan::OptimisePath(std::vector<KDPoint> &path) {
  std::vector<KDPoint> optimised_path = path;
  bool is_path_improved = true;
  while (is_path_improved) {
    is_path_improved = false;
    if (optimised_path.size() < 3) {
      std::cout << "SIZE < 3 ->Path optimisation done!" << std::endl;
      return optimised_path;
    }
    // Node we start looking from
    KDPoint current_node = optimised_path.rbegin()[1];
    // Node we want to reduce the cost
    KDPoint node_to_opt = optimised_path.back();

    // iterates from gate to start
    while (current_node != _map_info->pt_start) {
      int current_idx =
          std::find_if(optimised_path.begin(), optimised_path.end(),
                       [&](KDPoint &p) { return p == current_node; }) -
          optimised_path.begin();
      // get parent of the current node
      KDPoint node_parent = optimised_path[current_idx - 1];
      // checks if node_to_opt is a victim
      auto it =
          std::find_if(_map_info->_victims.begin(), _map_info->_victims.end(),
                       [&](std::tuple<KDPoint, double> &victim) {
                         return (std::get<0>(victim) == current_node);
                       });
      std::vector<KDPoint> segment = {node_parent, node_to_opt};
      if (!_map_info->Collision(segment) && it == _map_info->_victims.end()) {
        optimised_path.erase(optimised_path.begin() + current_idx);
        is_path_improved = true;
      } else {
        node_to_opt = current_node;
      }
      current_node = node_parent;
    }
  }
  std::cout << "Path optimisation done!" << std::endl;

  return optimised_path;
}
