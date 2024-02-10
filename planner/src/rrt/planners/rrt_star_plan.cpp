#include "planner/rrt/planners/rrt_star_plan.hpp"

#include <unistd.h>
RRTStarPlan::RRTStarPlan(std::shared_ptr<MapInfo> &map_info)
    : MotionPlanning(map_info), _rrt(map_info->_victims, this->seed) {
  rclcpp::sleep_for(std::chrono::seconds(1));
  _display = map_info->_show_graphics;
  _rrt.set_root(MotionPlanning::_pt_start);
}

KDPoint RRTStarPlan::_GenerateRandPoint(int iter) {
  // unsigned seed =
  // std::chrono::system_clock::now().time_since_epoch().count();
  int num_victims = MotionPlanning::_map_info->_victims.size();
  std::uniform_int_distribution<int> dis_s(0, 100);
  // Epsilon greedy sampling
  int extracted = dis_s(_rrt.generator);
  if (extracted < 50 - iter * 0.02) {
    int idx =
        std::uniform_int_distribution<int>(0, num_victims - 1)(_rrt.generator);
    return std::get<0>(MotionPlanning::_map_info->_victims[idx]);
  } else {
    if (extracted < 99.9 - iter * 0.02) {
      // sample from the square embedding the map
      std::uniform_real_distribution<> dis_x(
          (MotionPlanning::_map_info->min_x),
          (MotionPlanning::_map_info->max_x));
      std::uniform_real_distribution<> dis_y(
          (MotionPlanning::_map_info->min_y),
          (MotionPlanning::_map_info->max_y));
      while (true) {
        // TODO: check if double sampling works
        double x = dis_x(_rrt.generator);
        double y = dis_y(_rrt.generator);
        KDPoint p = {double(x), double(y)};
        if (!MotionPlanning::_map_info->Collision(p)) {
          return p;
        }
      }
    } else {
      // std::cout << "Selected end point with probability "
      //           << iter * 0.01 << std::endl;
      return MotionPlanning::_map_info->pt_end;
    }
  }
}

std::vector<KDPoint> RRTStarPlan::_ReconstrucPath(void) {
  std::vector<KDPoint> path;
  KDPoint p = MotionPlanning::_pt_end;
  while (p != MotionPlanning::_pt_start) {
    path.push_back(p);
    p = _rrt.GetParent(p);
  }
  path.push_back(p);
  std::reverse(path.begin(), path.end());

  // print the cost of every node
  for (size_t i = 0; i < _rrt._rrt.size(); i++) {
    KDPoint p = _rrt._rrt[i].first;
  }
  return path;
}

std::tuple<std::vector<KDPoint>, double> RRTStarPlan::run(void) {
  int iter = 0;
  auto startTime = std::chrono::high_resolution_clock::now();
  while (true) {
    // handles timeout returning empty path and infinite cost
    if (std::chrono::high_resolution_clock::now() - startTime >
        std::chrono::milliseconds(1000 *
                                  (int)MotionPlanning::_map_info->_timeout))
                                  {
      std::cout << "\033[0;31mTimeout\033[0m" << std::endl;
      std::vector<KDPoint> empty;
      return std::make_tuple(empty, std::numeric_limits<double>::infinity());
    }

    KDPoint q_rand = _GenerateRandPoint(iter);
    KDPoint q_near = _rrt.SearchNearestVertex(q_rand, iter);
    KDPoint q_new = _rrt.CalcNewPoint(q_near, q_rand);
    if (MotionPlanning::_map_info->Collision(q_new)) {
      continue;
    }
    // check that the new point is not already in the tree
    KDPoint p = q_near;
    bool already_visited = false;
    while (p != _rrt._root) {
      if (p == q_new) {
        already_visited = true;
        break;
      }
      p = _rrt.GetParent(p);
    }
    if (already_visited) {
      continue;
    }
    std::vector<KDPoint> branch;
    branch.push_back(q_near);
    branch.push_back(q_new);
    if (MotionPlanning::_map_info->Collision(branch)) {
      continue;
    }
    _rrt.Add(q_new, q_near);
    _rrt.Rewire(q_new, 2, [&](std::vector<KDPoint> &branch) {
      return MotionPlanning::_map_info->Collision(branch);
    });

    if (MotionPlanning::_display) {
      MotionPlanning::_map_info->set_rrt(_rrt, 0, q_rand);
    }
    // convergence condition
    if (Distance(q_new, MotionPlanning::_pt_end) < 0.1) {
      // the last point is not the end point -> we need to add it
      if (q_new != MotionPlanning::_pt_end) {
        _rrt.Add(MotionPlanning::_pt_end, q_new);
      }
      while (true) {
        auto parent = _rrt.GetParent(MotionPlanning::_pt_end);
        if (_rrt.PathOptimisation(parent, MotionPlanning::_pt_end,
                                  [&](std::vector<KDPoint> &path) {
                                    return MotionPlanning::_map_info->Collision(
                                        path);
                                  }) == false) {
          break;
        }
      }
      std::cout << "Found path with cost "
                << _rrt.Cost(MotionPlanning::_pt_end, true) << std::endl;
      std::tuple<std::vector<KDPoint>, double> final_path_cost =
          std::make_tuple(_ReconstrucPath(),
                          _rrt.Cost((MotionPlanning::_pt_end), true));
      return final_path_cost;
    }
    iter++;
  }
}