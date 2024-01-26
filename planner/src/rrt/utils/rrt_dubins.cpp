#include "planner/rrt/utils/rrt_dubins.hpp"

void RRTDubins::set_root(KDPoint &p) {
  _root.assign(p.begin(), p.end());
  _rrt.push_back(std::make_tuple(_root, 0, std::vector<std::vector<double>>(),
                                 std::vector<KDPoint>()));
}

KDPoint RRTDubins::SearchNearestVertex(KDPoint &q_rand) {
  std::vector<double> d;
  for (auto tuple : _rrt) {
    // d.push_back(Distance(pair.first, q_rand));
    d.push_back(Distance(std::get<0>(tuple), q_rand));
  }
  int i = std::min_element(d.begin(), d.end()) - d.begin();
  return std::get<0>(_rrt[i]);
}

void RRTDubins::Add(KDPoint &q_new, KDPoint &q_near, SymbolicPath &sym_path,
                    std::vector<KDPoint> &path) {
  int i = std::find_if(
              _rrt.begin(), _rrt.end(),
              [&](std::tuple<KDPoint, int, SymbolicPath, std::vector<KDPoint>>
                      &tuple) { return (std::get<0>(tuple) == q_near); }) -
          _rrt.begin();

  // std::cout << "Parent of \t(" << q_new[0] << ", " << q_new[1] << ") is ("
  //           << q_near[0] << ", " << q_near[1] << ") \t|| i = " << i
  //           << "\t rrt_size: " << _rrt.size() << std::endl;
  // _rrt.push_back(std::make_tuple(q_new, i, sym_path, path));
  _rrt.push_back(std::make_tuple(q_new, i, sym_path, path));
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

double RRTDubins::Cost(KDPoint p, double radius) {
  double cost = 0.0;
  KDPoint q = p;
  while (q != _root) {
    std::tuple<KDPoint, int, SymbolicPath, std::vector<KDPoint>> f =
        GetParent(q);
    for (auto path : std::get<2>(f)) {
      cost += (path[0] == 's' ? path[1] : path[1] * radius);
    }
    q = std::get<0>(f);
  }
  return cost;
}

void RRTDubins::Rewire(
    KDPoint &q_new, double r,
    std::function<bool(std::vector<KDPoint> &path)> DubinsCollision,
    double dubins_radius) {
  std::vector<KDPoint> nears;
  auto it_p = std::find_if(
      _rrt.begin(), _rrt.end(),
      [&](std::tuple<KDPoint, int, SymbolicPath, std::vector<KDPoint>> &tuple) {
        return (std::get<0>(tuple) == q_new);
      });
  // fill nears
  std::for_each(
      _rrt.begin(), _rrt.end(),
      [&](std::tuple<KDPoint, int, SymbolicPath, std::vector<KDPoint>> &tuple) {
        if ((std::get<0>(tuple) != q_new) &&
            (Distance(std::get<0>(tuple), q_new) < r)) {
          nears.push_back(std::get<0>(tuple));
        }
      });

  for (auto q : nears) {
    // X, Y, Cost, Symbolic Path
    auto dubins_best_path =
        get_dubins_best_path_and_cost(q, q_new, dubins_radius, 0.1);

    // Check collision
    // Path new_path = std::make_tuple(std::get<0>(dubins_best_path),
    //                                 std::get<1>(dubins_best_path));
    std::vector<KDPoint> new_path = std::get<0>(dubins_best_path);

    // compute distance given the symbolic path
    double distance = 0.0;
    for (auto &p : std::get<2>(dubins_best_path)) {
      if (p[0] == 's') {
        distance += p[1];
      } else {
        distance += p[1] * dubins_radius;
      }
    }
    // Check cost improvement
    if (Cost(q, dubins_radius) + distance < Cost(q_new, dubins_radius)) {
      // Check collision of the new path
      if (DubinsCollision(new_path)) {
        continue;
      }
      // Rewire nodes with their new parent
      int idx =
          std::find_if(
              _rrt.begin(), _rrt.end(),
              [&](std::tuple<KDPoint, int, SymbolicPath, std::vector<KDPoint>>
                      &tuple) { return (std::get<0>(tuple) == q); }) -
          _rrt.begin();
      // it_p->second = idx;
      std::get<1>(*it_p) = idx;
      // Update symbolic path
      std::get<2>(*it_p) = std::get<2>(dubins_best_path);
      // Update path
      std::get<3>(*it_p) = new_path;
    }
  }
  // for (auto q : nears)
  // {
  //   auto dubins_best_path =
  //       get_dubins_best_path_and_cost(q_new, q, dubins_radius, 0.1);

  //   // Check collision
  //   std::reverse(std::get<0>(dubins_best_path).begin(),
  //                std::get<0>(dubins_best_path).end());
  //   std::reverse(std::get<1>(dubins_best_path).begin(),
  //                std::get<1>(dubins_best_path).end());
  //   std::tuple<std::vector<double>, std::vector<double>> new_path =
  //       std::make_tuple(std::get<0>(dubins_best_path),
  //                       std::get<1>(dubins_best_path));

  //   double distance = 0.0;
  //   for (auto &p : std::get<3>(dubins_best_path))
  //   {
  //     if (p[0] == 's')
  //     {
  //       distance += p[1];
  //     }
  //     else
  //     {
  //       distance += p[1] * dubins_radius;
  //     }
  //   }
  //   if (abs(Cost(q_new, dubins_radius) + distance - Cost(q, dubins_radius)) >
  //   1e-6)
  //   {
  //     std::cout << "SERVE A QUALCOSA" << std::endl;
  //     auto it_pt = std::find_if(
  //         _rrt.begin(), _rrt.end(),
  //         [&](std::tuple<KDPoint, int, SymbolicPath, Path> &tuple)
  //         {
  //           return (std::get<0>(tuple) == q);
  //         });
  //     // it_pt->second = int(it_p - _rrt.begin());
  //     std::get<1>(*it_pt) = int(it_p - _rrt.begin());

  //     // it_p->second = idx;
  //     // Update symbolic path
  //     std::get<2>(*it_p) = std::get<3>(dubins_best_path);
  //     // Update path
  //     std::get<3>(*it_p) = new_path;
  //   }
  // }
}
