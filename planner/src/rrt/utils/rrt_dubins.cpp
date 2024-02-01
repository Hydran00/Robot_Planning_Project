#include "planner/rrt/utils/rrt_dubins.hpp"
// #include <limits>
#include <unistd.h>

#define VELOCITY 0.2
#define TIME_LIMIT 500000.0

void RRTDubins::set_root(KDPoint &p) {
  _root.assign(p.begin(), p.end());
  _rrt.push_back(std::make_tuple(_root, 0, std::vector<std::vector<double>>(),
                                 std::vector<KDPoint>()));
}

std::tuple<KDPoint, int, SymbolicPath, std::vector<KDPoint>>
RRTDubins::SearchNearestVertex(KDPoint &q_rand, double radius, int iter) {
  std::vector<double> d;
  // extract random point with prob 0.05
  unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
  std::default_random_engine generator(seed);
  std::uniform_int_distribution<int> epsilon_greedy_prob(0, 100);
  if (epsilon_greedy_prob(generator) < (float)0.01 * iter) {
    std::uniform_int_distribution<int> dis_s(0, _rrt.size() - 1);
    int idx = dis_s(generator);
    std::cout << "Selected nearest random node with prob " << (float)0.01 * iter
              << std::endl;
    return _rrt[idx];
  }
  for (auto node : _rrt) {
    double distance = sqrt(pow(q_rand[0] - std::get<0>(node)[0], 2) +
                           pow(q_rand[1] - std::get<0>(node)[1], 2));
    distance = 0.0;

    if (Cost(node, radius, false) > VELOCITY * TIME_LIMIT) {
      // std::cout << "Node too far" << std::endl;
      // exclude nodes that are too far to be reached
      d.push_back(std::numeric_limits<double>::infinity());
    } else {
      d.push_back(distance + 0.95 * Cost(node, radius, true));
    }
  }
  int i = std::min_element(d.begin(), d.end()) - d.begin();
  return _rrt[i];
}

std::tuple<KDPoint, int, SymbolicPath, std::vector<KDPoint>> RRTDubins::Add(
    KDPoint &q_new,
    std::tuple<KDPoint, int, SymbolicPath, std::vector<KDPoint>> &q_near,
    SymbolicPath &sym_path, std::vector<KDPoint> &path) {
  int i = std::find_if(_rrt.begin(), _rrt.end(),
                       [&](std::tuple<KDPoint, int, SymbolicPath,
                                      std::vector<KDPoint>> &tuple) {
                         return (std::get<0>(tuple) == std::get<0>(q_near));
                       }) -
          _rrt.begin();
  _rrt.push_back(std::make_tuple(q_new, i, sym_path, path));
  return _rrt.back();
}

std::tuple<KDPoint, int, SymbolicPath, std::vector<KDPoint>>
RRTDubins::GetParent(KDPoint &p) {
  auto it = std::find_if(
      _rrt.begin(), _rrt.end(),
      [&](std::tuple<KDPoint, int, SymbolicPath, std::vector<KDPoint>> &tuple) {
        return (std::get<0>(tuple) == p);
      });

  // return _rrt[it->second].first;
  return _rrt[std::get<1>(*it)];
}

std::vector<KDPoint> RRTDubins::GetPointPath(KDPoint &p) {
  auto it = std::find_if(
      _rrt.begin(), _rrt.end(),
      [&](std::tuple<KDPoint, int, SymbolicPath, std::vector<KDPoint>> &tuple) {
        return (std::get<0>(tuple) == p);
      });
  return std::get<3>(*it);
}

double RRTDubins::GetPathLength(SymbolicPath &sym_path, double radius) {
  double length = 0.0;
  for (auto path : sym_path) {
    length += (path[0] == 's' ? path[1] : path[1] * radius);
  }
  return length;
}
double RRTDubins::Cost(
    std::tuple<KDPoint, int, SymbolicPath, std::vector<KDPoint>> &node,
    double radius, bool consider_victims) {
  double cost = 0.0;
  std::vector<std::tuple<KDPoint, double>> victims_list = victims;
  std::tuple<KDPoint, int, SymbolicPath, std::vector<KDPoint>> q = node;

  while (std::get<0>(q) != _root) {
    cost += GetPathLength(std::get<2>(q), radius);

    if (consider_victims) {
      auto it = std::find_if(victims_list.begin(), victims_list.end(),
                             [&](std::tuple<KDPoint, double> &victim) {
                               return (std::get<0>(victim) == std::get<0>(q) &&
                                       std::get<0>(victim) == std::get<0>(q));
                             });
      if (it != victims_list.end()) {
        cost -= std::get<1>(*it);
        victims_list.erase(it);
      }
    } else {
      std::cout << "Adding cost " << GetPathLength(std::get<2>(q), radius)
                << std::endl;
    }
    q = GetParent(std::get<0>(q));
  }
  std::cout << "-------------------" << std::endl;
  return cost;
}

void RRTDubins::Rewire(
    std::tuple<KDPoint, int, SymbolicPath, std::vector<KDPoint>> &q_new,
    double r, std::function<bool(std::vector<KDPoint> &path)> DubinsCollision,
    double dubins_radius) {
  std::vector<std::tuple<KDPoint, int, SymbolicPath, std::vector<KDPoint>>>
      nears;

  auto it_p = std::find_if(
      _rrt.begin(), _rrt.end(),
      [&](std::tuple<KDPoint, int, SymbolicPath, std::vector<KDPoint>> &node) {
        return (std::get<0>(node) == std::get<0>(q_new));
      });

  std::for_each(
      _rrt.begin(), _rrt.end(),
      [&](std::tuple<KDPoint, int, SymbolicPath, std::vector<KDPoint>> &node) {
        if ((std::get<0>(node) != std::get<0>(q_new)) &&
            (Distance(std::get<0>(node), std::get<0>(q_new)) < r)) {
          nears.push_back(node);
        }
      });
  for (auto q : nears) {
    // std::vector<KDPoint> Path, double cost, std::vector<std::vector<double>
    // Symbolic Path
    auto dubins_best_path = get_dubins_best_path_and_cost(
        std::get<0>(q), std::get<0>(q_new), dubins_radius, 0.1);

    // compute the length of the last segment of the new path
    double last_segment_distance =
        GetPathLength(std::get<2>(dubins_best_path), dubins_radius);

    // avoid rewiring if the new total path is too long to be travelled
    double distance = Cost(q, dubins_radius, false) + last_segment_distance;
    if (distance > VELOCITY * TIME_LIMIT) {
      continue;
    }

    // check if q_new is victim
    auto it = std::find_if(victims.begin(), victims.end(),
                           [&](std::tuple<KDPoint, double> &victim) {
                             return (std::get<0>(victim) == std::get<0>(q_new));
                           });
    if (it != victims.end()) {
      last_segment_distance -= std::get<1>(*it);
    }

    // Check cost improvement
    if (Cost(q, dubins_radius, true) + last_segment_distance <
        Cost(q_new, dubins_radius, true)) {
      // Check collision of the new path
      if (DubinsCollision(std::get<0>(dubins_best_path))) {
        continue;
      }
      // Rewire nodes with their new parent
      int idx = std::find_if(_rrt.begin(), _rrt.end(),
                             [&](std::tuple<KDPoint, int, SymbolicPath,
                                            std::vector<KDPoint>> &tuple) {
                               return (std::get<0>(tuple) == std::get<0>(q));
                             }) -
                _rrt.begin();
      // Update parent
      std::get<1>(*it_p) = idx;
      // Update symbolic path
      std::get<2>(*it_p) = std::get<2>(dubins_best_path);
      // Update path
      std::get<3>(*it_p) = std::get<0>(dubins_best_path);
    }
  }
}
// //    def PathOptimization(self, node):
//         direct_cost_new = 0.0
//         node_end = self.x_goal

//         while node.parent:
//             node_parent = node.parent
//             if not self.utils.is_collision(node_parent, node_end):
//                 node_end.parent = node_parent
//             else:
//                 direct_cost_new += self.Line(node, node_end)
//                 node_end = node

//             node = node_parent

//         if direct_cost_new < self.direct_cost_old:
//             self.direct_cost_old = direct_cost_new
//             self.UpdateBeacons()

bool RRTDubins::PathOptimisation(
    std::tuple<KDPoint, int, SymbolicPath, std::vector<KDPoint>> &current_node_,
    std::tuple<KDPoint, int, SymbolicPath, std::vector<KDPoint>> &node_end,
    std::function<bool(std::vector<KDPoint> &path)> DubinsCollision,
    double dubins_radius) {
  std::cout << "\033[1;35mStart optimising!\033[0m" << std::endl;

  bool is_path_improved = false;
  double direct_cost_new = 0.0;

  // Node we start looking from
  auto current_node = current_node_;

  // Node we want to reduce the cost
  auto node_to_opt = node_end;


  std::cout << "Root is "<<  _root[0] << ", " << _root[1] << std::endl;
  std::cout << "Current node is "<<  std::get<0>(current_node)[0] << ", " << std::get<0>(current_node)[1] << std::endl;
  if (std::get<0>(current_node) == _root) {
    std::cout << "Root coincide: " << (std::get<0>(current_node) == _root)
              << std::endl;
    return false;
  }
  // iterates from node_new to root
  while (std::get<0>(current_node) != _root) {
    // get parent of the current node
    auto node_parent = GetParent(std::get<0>(current_node));

    // generate Dubins from node_parent to node_end
    auto dubins_opt_path1 = get_dubins_best_path_and_cost(
        std::get<0>(node_parent), std::get<0>(node_to_opt), dubins_radius, 0.1);

    std::cout << "Trying to optimise path from " << std::get<0>(node_parent)[0]
              << ", " << std::get<0>(node_parent)[1] << " to "
              << std::get<0>(node_to_opt)[0] << ", "
              << std::get<0>(node_to_opt)[1] << std::endl;
    double new_cost =
        Cost(node_parent, dubins_radius, false) +
        GetPathLength(std::get<2>(dubins_opt_path1), dubins_radius);

    if (new_cost < Cost(node_to_opt, dubins_radius, false) + 0.04 &&
        !DubinsCollision(std::get<0>(dubins_opt_path1))) {
      // if the Dubins is collision free, we can optimise the path
      std::cout << "\033[1;32mOptimisation found!\033[0m" << std::endl;

      auto it_node_to_opt = std::find_if(
          _rrt.begin(), _rrt.end(),
          [&](std::tuple<KDPoint, int, SymbolicPath, std::vector<KDPoint>>
                  &tuple) {
            return (std::get<0>(tuple) == std::get<0>(node_to_opt));
          });

      std::get<1>(*it_node_to_opt) = std::get<1>(current_node);
      std::get<2>(*it_node_to_opt) = std::get<2>(dubins_opt_path1);
      std::get<3>(*it_node_to_opt) = std::get<0>(dubins_opt_path1);

      // std::cout << "Cost was " << Cost(node_to_opt, dubins_radius, false)
      //           << " and becomes " << new_cost << std::endl;
      // // update the parent of the node we want to optimise
      // std::cout << "Parent was " << std::get<1>(node_to_opt) << " and becomes
      // "
      //           << std::get<1>(current_node) << std::endl;
      is_path_improved = true;
    } else {
      std::cout << "\033[1;33mOptimisation not found!--> Recursion\033[0m"
                << std::endl;
      node_to_opt = current_node;
    }
    current_node = node_parent;
  }
  return is_path_improved;
}

// def UpdateBeacons(self):
//     node = self.x_goal
//     beacons = []

//     while node.parent:
//         near_vertex = [v for v in self.obs_vertex
//                        if (node.x - v[0]) ** 2 + (node.y - v[1]) ** 2 < 9]
//         if len(near_vertex) > 0:
//             for v in near_vertex:
//                 beacons.append(v)

//         node = node.parent

//     self.beacons = beacons
void RRTDubins::UpdateBeacons(KDPoint &q_end){};
//   KDPoint node = q_end;
//   std::vector<KDPoint> beacons;
//   while (node != _root) {
//     std::vector<KDPoint> near_vertex;
//     for (auto v : MotionPlanning::_map_info->GetObstacleVertex()) {
//       if (Distance(node, v) < 3.0) {
//         near_vertex.push_back(v);
//       }
//     }
//     if (near_vertex.size() > 0) {
//       for (auto v : near_vertex) {
//         beacons.push_back(v);
//       }
//     }
//     node = GetParent(node);
//   }
//   _beacons = beacons;
// }
