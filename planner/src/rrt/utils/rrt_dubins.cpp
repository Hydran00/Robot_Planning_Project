#include "planner/rrt/utils/rrt_dubins.hpp"
// #include <limits>
#include <unistd.h>

#define VELOCITY 0.2
#define TIME_LIMIT 25.0

void RRTDubins::set_root(KDPoint &p) {
  _root.assign(p.begin(), p.end());
  _rrt.push_back(std::make_tuple(_root, 0, std::vector<std::vector<double>>(),
                                 std::vector<KDPoint>()));
}

std::tuple<KDPoint, int, SymbolicPath, std::vector<KDPoint>>
RRTDubins::SearchNearestVertex(KDPoint &q_rand, double radius, int iter) {
  std::vector<double> d;
  // extract random point with prob 0.05
  // unsigned seed =
  // std::chrono::system_clock::now().time_since_epoch().count();
  // std::default_random_engine generator(seed);
  std::uniform_int_distribution<int> epsilon_greedy_prob(0, 100);
  if (epsilon_greedy_prob(generator) < (float)0.01 * iter) {
    std::uniform_int_distribution<int> dis_s(0, _rrt.size() - 1);
    int idx = dis_s(generator);
    return _rrt[idx];
  }
  double distance;
  for (auto node : _rrt) {
    distance = sqrt(pow(q_rand[0] - std::get<0>(node)[0], 2) +
                    pow(q_rand[1] - std::get<0>(node)[1], 2));

    if (Cost(node, radius, false) + distance > VELOCITY * TIME_LIMIT) {
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
    }
    q = GetParent(std::get<0>(q));
  }
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

  // check if q_new is victim
  double qnew_victim_discount = 0.0;
  auto it = std::find_if(victims.begin(), victims.end(),
                         [&](std::tuple<KDPoint, double> &victim) {
                           return (std::get<0>(victim) == std::get<0>(q_new));
                         });
  if (it != victims.end()) {
    qnew_victim_discount = -std::get<1>(*it);
  }

  for (auto pt : nears) {
    // std::vector<KDPoint> Path, double cost, std::vector<std::vector<double>
    // Symbolic Path
    auto dubins_best_path = get_dubins_best_path_and_cost(
        std::get<0>(pt), std::get<0>(q_new), dubins_radius, 0.1);

    // compute the length of the last segment of the new path
    double last_segment_length =
        GetPathLength(std::get<2>(dubins_best_path), dubins_radius);

    // avoid rewiring if the new total path is too long to be travelled
    double distance = Cost(pt, dubins_radius, false) + last_segment_length;
    if (distance > VELOCITY * TIME_LIMIT) {
      continue;
    }

    // Check cost improvement
    if (Cost(pt, dubins_radius, true) + last_segment_length +
            qnew_victim_discount <
        Cost(q_new, dubins_radius, true)) {
      // Check collision of the new path
      if (DubinsCollision(std::get<0>(dubins_best_path))) {
        continue;
      }
      // Rewire q_new with its new parent
      int idx = std::find_if(_rrt.begin(), _rrt.end(),
                             [&](std::tuple<KDPoint, int, SymbolicPath,
                                            std::vector<KDPoint>> &tuple) {
                               return (std::get<0>(tuple) == std::get<0>(pt));
                             }) -
                _rrt.begin();
      // Update parent
      std::get<1>(*it_p) = idx;
      // std::get<1>(q_new) = idx;
      // Update symbolic path
      std::get<2>(*it_p) = std::get<2>(dubins_best_path);
      // std::get<2>(q_new) = std::get<2>(dubins_best_path);
      // Update path
      std::get<3>(*it_p) = std::get<0>(dubins_best_path);
      // std::get<3>(q_new) = std::get<0>(dubins_best_path);
    }
  }

  // for (auto pt : nears) {
  //   // checks if pt is a victim
  //   auto it_vict_pt = std::find_if(victims.begin(), victims.end(),
  //                                  [&](std::tuple<KDPoint, double> &victim) {
  //                                    return (std::get<0>(victim) == pt);
  //                                  });
  //   double victim_discount_pt = 0.0;
  //   if (it_vict_pt != victims.end()) {
  //     victim_discount_pt = -std::get<1>(*it_vict_pt);
  //   }

  //   auto dubins_best_path = get_dubins_best_path_and_cost(
  //       std::get<0>(q_new), std::get<0>(pt), dubins_radius, 0.1);

  //   // compute the length of the last segment of the new path
  //   double last_segment_length =
  //       GetPathLength(std::get<2>(dubins_best_path), dubins_radius);

  //   // avoid rewiring if the new total path is too long to be travelled
  //   double distance = Cost(pt, dubins_radius, false) + last_segment_length;
  //   if (distance > VELOCITY * TIME_LIMIT) {
  //     continue;
  //   }

  //   // Check cost improvement
  //   if (Cost(pt, dubins_radius, true) + last_segment_length +
  //           qnew_victim_discount <
  //       Cost(q_new, dubins_radius, true)) {
  //     // Check collision of the new path
  //     if (DubinsCollision(std::get<0>(dubins_best_path))) {
  //       continue;
  //     }
  //     // Rewire q_new with its new parent
  //     int idx = std::find_if(_rrt.begin(), _rrt.end(),
  //                            [&](std::tuple<KDPoint, int, SymbolicPath,
  //                                           std::vector<KDPoint>> &tuple) {
  //                              return (std::get<0>(tuple) == std::get<0>(pt));
  //                            }) -
  //               _rrt.begin();
  //     // Update parent
  //     std::get<1>(*it_p) = idx;
  //     // std::get<1>(q_new) = idx;
  //     // Update symbolic path
  //     std::get<2>(*it_p) = std::get<2>(dubins_best_path);
  //     // std::get<2>(q_new) = std::get<2>(dubins_best_path);
  //     // Update path
  //     std::get<3>(*it_p) = std::get<0>(dubins_best_path);
  //     // std::get<3>(q_new) = std::get<0>(dubins_best_path);
  //   }
  // }
}

bool RRTDubins::PathOptimisation(
    std::tuple<KDPoint, int, SymbolicPath, std::vector<KDPoint>> &current_node_,
    std::tuple<KDPoint, int, SymbolicPath, std::vector<KDPoint>> &node_end,
    std::function<bool(std::vector<KDPoint> &path)> DubinsCollision,
    double dubins_radius) {
  // std::cout << "\033[1;35mStart optimising!\033[0m" << std::endl;
  // std::cout << "---------------------" << std::endl;

  bool is_path_improved = false;

  // Node we start looking from
  auto current_node = current_node_;

  // Node we want to reduce the cost
  auto node_to_opt = node_end;

  // iterates from node_new to root
  while (std::get<0>(current_node) != _root) {
    // get parent of the current node
    auto node_parent = GetParent(std::get<0>(current_node));

    // generate Dubins from node_parent to node_end
    auto dubins_opt_path1 = get_dubins_best_path_and_cost(
        std::get<0>(node_parent), std::get<0>(node_to_opt), dubins_radius, 0.1);

    double new_cost =
        Cost(node_parent, dubins_radius, true) +
        GetPathLength(std::get<2>(dubins_opt_path1), dubins_radius);
    // checks if node_to_opt is a victim
    auto it =
        std::find_if(victims.begin(), victims.end(),
                     [&](std::tuple<KDPoint, double> &victim) {
                       return (std::get<0>(victim) == std::get<0>(node_to_opt));
                     });
    if (it != victims.end()) {
      new_cost -= std::get<1>(*it);
    }

    // std::cout << "Trying to skip node " << std::get<0>(current_node)[0] << "
    // "
    //           << std::get<0>(current_node)[1] << " and connect "
    //           << std::get<0>(node_parent)[0] << " "
    //           << std::get<0>(node_parent)[1] << " to "
    //           << std::get<0>(node_to_opt)[0] << " "
    //           << std::get<0>(node_to_opt)[1] << std::endl;
    if (new_cost < Cost(node_to_opt, dubins_radius, true) &&
        !DubinsCollision(std::get<0>(dubins_opt_path1))) {
      // if the Dubins is collision free, we can optimise the path
      // std::cout << "\033[1;32mOptimisation found!\033[0m" << std::endl;

      auto it_node_to_opt = std::find_if(
          _rrt.begin(), _rrt.end(),
          [&](std::tuple<KDPoint, int, SymbolicPath, std::vector<KDPoint>>
                  &tuple) {
            return (std::get<0>(tuple) == std::get<0>(node_to_opt));
          });

      std::get<1>(*it_node_to_opt) = std::get<1>(current_node);
      std::get<2>(*it_node_to_opt) = std::get<2>(dubins_opt_path1);
      std::get<3>(*it_node_to_opt) = std::get<0>(dubins_opt_path1);

      is_path_improved = true;
    } else {
      // std::cout << "\033[1;33mOptimisation not found!\033[0m" << std::endl;
      // std::cout << "Reason-> Cost improvement:"
      //           << ((new_cost < Cost(node_to_opt, dubins_radius, true))
      //                   ? "true"
      //                   : "false")
      //           << " | Path free: "
      //           << (!DubinsCollision(std::get<0>(dubins_opt_path1)) ? "true"
      //                                                               :
      //                                                               "false")
      //           << std::endl;
      // std::cout << "Costs are : " << new_cost << " and "
      //           << Cost(node_to_opt, dubins_radius, false) << std::endl;
      node_to_opt = current_node;
    }
    // std::cout << "---------------------" << std::endl;
    current_node = node_parent;
  }
  return is_path_improved;
}
