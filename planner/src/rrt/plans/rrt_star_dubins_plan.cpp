#include "planner/rrt/rrt_star_dubins_plan.hpp"

#include "planner/dubins/dubins.h"

RRTStarDubinsPlan::RRTStarDubinsPlan(std::shared_ptr<MapInfo> &map_info,
                                     double radius)
    : MotionPlanning(map_info) {
  _display = map_info->_show_graphics;
  _rrt.set_root(MotionPlanning::_pt_start);
  _radius = radius;
}

KDPoint RRTStarDubinsPlan::_GenerateRandPoint(void) {
  unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
  std::default_random_engine generator(seed);
  std::uniform_int_distribution<int> dis_s(0, 9);
  // Epsilon greedy sampling
  if (dis_s(generator) < 2) {
    return MotionPlanning::_pt_end;
  } else {
    // sample from the square embedding the map
    std::uniform_real_distribution<> dis_x((MotionPlanning::_map_info->min_x),
                                           (MotionPlanning::_map_info->max_x));
    std::uniform_real_distribution<> dis_y((MotionPlanning::_map_info->min_y),
                                           (MotionPlanning::_map_info->max_y));
    std::uniform_real_distribution<double> dis_yaw(0.0, 2 * M_PI);
    while (true) {
      // TODO: check if double sampling works
      double x = dis_x(generator);
      double y = dis_y(generator);
      double theta = dis_yaw(generator);
      KDPoint p = {double(x), double(y), double(theta)};
      if (!MotionPlanning::_map_info->Collision(p)) {
        return p;
      }
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
  std::reverse(path.begin(), path.end());
  return path;
}

std::vector<KDPoint> RRTStarDubinsPlan::run(void) {
  // Linestring linestring;
  int nodes_counter = 0;
  // uint improvements = 0;
  KDPoint p;
  while (true) {
    // linestring.clear();
    p.clear();
    KDPoint q_rand = _GenerateRandPoint();
    KDPoint q_near = _rrt.SearchNearestVertex(q_rand);

    // Returns tuple with (vector<KDPoint>, Cost, Symbolic Path)
    auto dubins_best_path =
        get_dubins_best_path_and_cost(q_near, q_rand, _radius, 0.1);
    // if (std::get<0>(dubins_best_path).size() <= 2) {

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
        q_near, 15.0,
        [&](std::vector<KDPoint> &path) {
          return MotionPlanning::_map_info->Collision(path);
        },
        _radius);

    if (MotionPlanning::_display) {
      MotionPlanning::_map_info->set_rrt_dubins(_rrt);
    }

    if (Distance(q_rand, MotionPlanning::_pt_end) < 0.1) {
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
    std::cout << "Number of nodes: " << nodes_counter << std::endl;
    std::cout << "---------------------" << std::endl;
  }
}
