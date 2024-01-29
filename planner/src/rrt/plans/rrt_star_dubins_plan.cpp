#include "planner/rrt/rrt_star_dubins_plan.hpp"

#include "planner/dubins/dubins.h"

RRTStarDubinsPlan::RRTStarDubinsPlan(std::shared_ptr<MapInfo> &map_info,
                                     double radius)
    : MotionPlanning(map_info), _rrt(map_info->_victims) {
  _display = map_info->_show_graphics;
  _rrt.set_root(MotionPlanning::_pt_start);
  _radius = radius;
}

// KDPoint RRTStarDubinsPlan::_GenerateRandPoint(void) {
//   unsigned seed =
//   std::chrono::system_clock::now().time_since_epoch().count();
//   std::default_random_engine generator(seed);
//   std::uniform_int_distribution<int> dis_s(0, 9);
//   // Epsilon greedy sampling
//   if (dis_s(generator) < 2) {
//     return MotionPlanning::_pt_end;
//   } else {
//     // sample from the square embedding the map
//     std::uniform_real_distribution<>
//     dis_x((MotionPlanning::_map_info->min_x),
//                                            (MotionPlanning::_map_info->max_x));
//     std::uniform_real_distribution<>
//     dis_y((MotionPlanning::_map_info->min_y),
//                                            (MotionPlanning::_map_info->max_y));
//     std::uniform_real_distribution<double> dis_yaw(0.0, 2 * M_PI);
//     while (true) {
//       // TODO: check if double sampling works
//       double x = dis_x(generator);
//       double y = dis_y(generator);
//       double theta = dis_yaw(generator);
//       KDPoint p = {double(x), double(y), double(theta)};
//       if (!MotionPlanning::_map_info->Collision(p)) {
//         return p;
//       }
//     }
//   }
// }
KDPoint RRTStarDubinsPlan::_GenerateRandPoint(int iter) {
  unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
  std::default_random_engine generator(seed);
  int num_victims = MotionPlanning::_map_info->_victims.size();
  std::uniform_int_distribution<int> dis_s(0, 100);
  // Epsilon greedy sampling
  int extracted = dis_s(generator);
  if (extracted < 80 - iter * 0.02) {
    int idx = std::uniform_int_distribution<int>(0, num_victims - 1)(generator);
    KDPoint p = std::get<0>(MotionPlanning::_map_info->_victims[idx]);
    std::uniform_real_distribution<double> dis_yaw(0.0, 2 * M_PI);
    p.push_back(dis_yaw(generator));
    // std::cout << "Selected: " << p[0] << ", " << p[1] << ", " << p[2]
    //           << std::endl;
    return p;
  } else {
    if (extracted < 99.9 - iter * 0.005) {
      // sample from the square embedding the map
      std::uniform_real_distribution<> dis_x(
          (MotionPlanning::_map_info->min_x),
          (MotionPlanning::_map_info->max_x));
      std::uniform_real_distribution<> dis_y(
          (MotionPlanning::_map_info->min_y),
          (MotionPlanning::_map_info->max_y));
      std::uniform_real_distribution<double> dis_yaw(0.0, 2 * M_PI);
      while (true) {
        double x = dis_x(generator);
        double y = dis_y(generator);
        double theta = dis_yaw(generator);
        KDPoint p = {double(x), double(y), double(theta)};
        if (!MotionPlanning::_map_info->Collision(p)) {
          return p;
        }
      }
    } else {
      // std::cout << "Sampling end point at iter " << iter
      //           << "| prob: " << iter * 0.01 << std::endl;
      std::uniform_real_distribution<double> dis_yaw(0.0, 2 * M_PI);
      KDPoint p = {MotionPlanning::_pt_end[0], MotionPlanning::_pt_end[1],
                   dis_yaw(generator)};
      return MotionPlanning::_pt_end;
    }
  }
}

std::vector<KDPoint> RRTStarDubinsPlan::_ReconstrucPath(void) {
  std::vector<KDPoint> path;
  KDPoint p = MotionPlanning::_pt_end;
  // extract last path given _pt_end
  auto last_path = _rrt.GetPointPath(p);
  // std::reverse(last_path.begin(), last_path.end());
  path.insert(path.begin(), last_path.begin(), last_path.end());
  while (p != MotionPlanning::_pt_start) {
    auto parent = _rrt.GetParent(p);
    std::vector<KDPoint> parent_path = std::get<3>(parent);
    path.insert(path.begin(), parent_path.begin(), parent_path.end());
    p = std::get<0>(parent);
  }
  // std::reverse(path.begin(), path.end());
  return path;
}

std::vector<KDPoint> RRTStarDubinsPlan::run(void) {
  int nodes_counter = 0;
  KDPoint p;
  int iter = 0;
  while (true) {
    iter++;
    p.clear();
    KDPoint q_rand = _GenerateRandPoint(iter);
    std::tuple<KDPoint, int, SymbolicPath, std::vector<KDPoint>> near_node =
        _rrt.SearchNearestVertex(q_rand, _radius);

    KDPoint q_near = std::get<0>(near_node);
    
    // check that the new point is not already in the tree
    KDPoint p = q_near;
    bool already_visited = false;
    while (p != _rrt._root) {
      if (p[0] == q_rand[0] && p[1] == q_rand[1]) {
        already_visited = true;
        break;
      }
      p = std::get<0>(_rrt.GetParent(p));
    }
    if (already_visited) {
      continue;
    }

    auto dubins_best_path =
        get_dubins_best_path_and_cost(q_near, q_rand, _radius, 0.1);
    // if (std:get<0>(dubins_best_path).size() <= 2) {

    //   continue;
    // }

    std::vector<KDPoint> new_path;
    for (size_t i = 0; i < std::get<0>(dubins_best_path).size(); i++) {
      p = std::get<0>(dubins_best_path)[i];
      new_path.push_back(p);

      p.clear();
    }

    // Check collisions
    if (MotionPlanning::_map_info->Collision(new_path)) {
      continue;
    }

    _rrt.Add(q_rand, q_near, std::get<2>(dubins_best_path), new_path);

    // TODO check radius->was 5.0
    _rrt.Rewire(
        near_node, 100.0,
        [&](std::vector<KDPoint> &path) {
          return MotionPlanning::_map_info->Collision(path);
        },
        _radius);

    if (MotionPlanning::_display) {
      MotionPlanning::_map_info->set_rrt_dubins(_rrt);
    }

    if (sqrt(pow(q_rand[0] - MotionPlanning::_pt_end[0], 2) +
             pow(q_rand[1] - MotionPlanning::_pt_end[1], 2)) < 1) {
      nodes_counter += 1;
      if (q_rand != MotionPlanning::_pt_end) {
        auto last_path = get_dubins_best_path_and_cost(
            q_near, MotionPlanning::_pt_end, _radius, 0.1);
        std::vector<KDPoint> last_segment;
        for (size_t i = 0; i < std::get<0>(last_path).size(); i++) {
          p = std::get<0>(last_path)[i];
          last_segment.push_back(p);
          p.clear();
        }
        _rrt.Add(MotionPlanning::_pt_end, q_near, std::get<2>(last_path),
                 last_segment);
      }
      return _ReconstrucPath();
    }
    nodes_counter += 1;
    std::cout << "Number of nodes: " << nodes_counter
              << "| prob of extractin end is " << (double)iter * 0.005
              << std::endl;
    std::cout << "---------------------" << std::endl;
  }
}
