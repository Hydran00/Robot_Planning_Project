#include "planner/rrt/planners/rrt_star_dubins_plan.hpp"

#include "planner/dubins/dubins.h"

RRTStarDubinsPlan::RRTStarDubinsPlan(std::shared_ptr<MapInfo> &map_info)
    : MotionPlanning(map_info), _rrt(map_info->_victims, this->seed) {
  _display = map_info->_show_graphics;
  _rrt.set_root(MotionPlanning::_pt_start);
  _radius = map_info->dubins_radius;
}

KDPoint RRTStarDubinsPlan::_GenerateRandPoint(int iter) {
  // unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
  // std::default_random_engine generator(seed);
  int num_victims = MotionPlanning::_map_info->_victims.size();
  std::uniform_int_distribution<int> dis_s(0, 100);
  // Epsilon greedy sampling
  int extracted = dis_s(_rrt.generator);
  if (extracted < 50 - iter * 0.02) {
    int idx = std::uniform_int_distribution<int>(0, num_victims - 1)(_rrt.generator);
    KDPoint p = std::get<0>(MotionPlanning::_map_info->_victims[idx]);
    std::uniform_real_distribution<double> dis_yaw(0.0, 2 * M_PI - EPSILON);
    p.push_back(dis_yaw(_rrt.generator));
    return p;
  } else {
    if (extracted < 99.9 - iter * 0.02) {
      // sample from the square embedding the map
      std::uniform_real_distribution<> dis_x(
          (MotionPlanning::_map_info->min_x),
          (MotionPlanning::_map_info->max_x));
      std::uniform_real_distribution<> dis_y(
          (MotionPlanning::_map_info->min_y),
          (MotionPlanning::_map_info->max_y));
      std::uniform_real_distribution<double> dis_yaw(0.0, 2 * M_PI - EPSILON);
      while (true) {
        double x = dis_x(_rrt.generator);
        double y = dis_y(_rrt.generator);
        double theta = dis_yaw(_rrt.generator);
        KDPoint p = {double(x), double(y), double(theta)};
        if (!MotionPlanning::_map_info->Collision(p)) {
          return p;
        }
      }
    } else {
      std::uniform_real_distribution<double> dis_yaw(0.0, 2 * M_PI - EPSILON);
      KDPoint p = {MotionPlanning::_pt_end[0], MotionPlanning::_pt_end[1],
                   dis_yaw(_rrt.generator)};
      return MotionPlanning::_pt_end;
    }
  }
}

std::vector<KDPoint> RRTStarDubinsPlan::_ReconstrucPath() {
  std::cout << "ENTERING RECONSTRUCT PATH" << std::endl;
  KDPoint p = MotionPlanning::_pt_end;
  std::vector<KDPoint> path;
  auto last_path = _rrt.GetPointPath(p);
  std::cout << "\t                          p:" << p[0] << ", " << p[1] << std::endl;
  std::cout << "\tPutting node. Starting from: (" << last_path[0][0] << ", " << last_path[0][1] << ")" << std::endl;
  int end_idx = last_path.size() - 1;
  std::cout << "\t                  Ending in: (" << last_path[end_idx][0] << ", " << last_path[end_idx][1] << ")" << std::endl;
  path.insert(path.begin(), last_path.begin(), last_path.end());
  while (MotionPlanning::_pt_start != p) {
    auto parent = _rrt.GetParent(p);
    std::vector<KDPoint> parent_path = std::get<3>(parent);
    std::cout << "\t                          p:" << p[0] << ", " << p[1] << std::endl;
    std::cout << "\tPutting node. Starting from: (" << parent_path[0][0] << ", " << parent_path[0][1] << ")" << std::endl;
    int end_idx = parent_path.size() - 1;
    std::cout << "\t                  Ending in: (" << parent_path[end_idx][0] << ", " << parent_path[end_idx][1] << ")\n" << std::endl;
    path.insert(path.begin(), parent_path.begin(), parent_path.end());
    p = std::get<0>(parent);
  }
  return path;
}

std::tuple<std::vector<KDPoint>, double> RRTStarDubinsPlan::run(void) {
  KDPoint p;
  int iter = 0;
  auto startTime = std::chrono::high_resolution_clock::now();
  while (true) {
    if (std::chrono::high_resolution_clock::now() - startTime >
        std::chrono::milliseconds(1000 *
                                  (int)MotionPlanning::_map_info->_timeout)) {
      std::cout << "\033[0;31mTimeout\033[0m" << std::endl;
      std::vector<KDPoint> empty;
      // return empy path and infinite cost
      return std::make_tuple(empty, std::numeric_limits<double>::infinity());
    }

    p.clear();
    KDPoint q_rand = _GenerateRandPoint(iter);
    std::tuple<KDPoint, int, SymbolicPath, std::vector<KDPoint>> near_node =
        _rrt.SearchNearestVertex(q_rand, _radius, iter);

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
      // std::cout << "Already visited" << std::endl;
      // std::cout << "---------------------" << std::endl;
      continue;
    }

    std::tuple<std::vector<KDPoint>, double, std::vector<std::vector<double>>>
        dubins_best_path =
            get_dubins_best_path_and_cost(q_near, q_rand, _radius, 0.1);

    std::vector<KDPoint> new_path = std::get<0>(dubins_best_path);

    // Checks collisions
    if (MotionPlanning::_map_info->Collision(new_path)) {
      continue;
    }
    // Gets the new node
    std::tuple<KDPoint, int, SymbolicPath, std::vector<KDPoint>> new_node =
        _rrt.Add(q_rand, near_node, std::get<2>(dubins_best_path), new_path);

    // The rewire function is the difference between RRT and RRT*
    _rrt.Rewire(
        new_node,100,
        [&](std::vector<KDPoint> &path) {
          return MotionPlanning::_map_info->Collision(path);
        },
        _radius);
    // Display the tree in Rviz2
    if (MotionPlanning::_display) {
      MotionPlanning::_map_info->set_rrt_dubins(_rrt);
    }

    // check if we are close to the end
    if (sqrt(pow(q_rand[0] - MotionPlanning::_pt_end[0], 2) +
             pow(q_rand[1] - MotionPlanning::_pt_end[1], 2)) < 0.2) {
      // the last point is not the end point -> we need to add it
      if (q_rand != MotionPlanning::_pt_end) {
        std::cout << "Node extracted is not pt_end" << std::endl;
        // rename for clarity
        q_near = q_rand;
        // generate last dubins path
        std::tuple<std::vector<KDPoint>, double,
                   std::vector<std::vector<double>>>
            dubins_best_path_end = get_dubins_best_path_and_cost(
                q_near, MotionPlanning::_pt_end, _radius, 0.1);
        // add the last path to the end point
        auto last_node = _rrt.Add(MotionPlanning::_pt_end, new_node,
                                  std::get<2>(dubins_best_path_end),
                                  std::get<0>(dubins_best_path_end));
      }
      // optimise the path
      double cost1 = _rrt.Cost(new_node, _radius, false);
      // }
      // keep optimising the path until it does not change anymore
      while (true) {
        auto parent = _rrt.GetParent(MotionPlanning::_pt_end);
        if (_rrt.PathOptimisation(
                parent, new_node,
                [&](std::vector<KDPoint> &path) {
                  return MotionPlanning::_map_info->Collision(path);
                },
                _radius) == false) {
          break;
        }
      };
      std::cout << "\n\nFinal cost of node " << std::get<0>(new_node)[0] << ", "
                << std::get<0>(new_node)[1] << " is " << cost1 << std::endl;
      std::cout << "Final cost after optimisation of node "
                << std::get<0>(new_node)[0] << ", " << std::get<0>(new_node)[1]
                << " is " << _rrt.Cost(new_node, _radius, false) << std::endl;
      std::tuple<std::vector<KDPoint>, double> final_path_cost =
          std::make_tuple(_ReconstrucPath(),
                          _rrt.Cost(new_node, _radius, true));
      return final_path_cost;
    }
    iter++;

  }
}
