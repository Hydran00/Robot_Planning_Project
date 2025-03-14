#include "planner/rrt/planners/rrt_plan.hpp"

RRTPlan::RRTPlan(std::shared_ptr<MapInfo> &map_info)
    : MotionPlanning(map_info), _rrt(map_info->_victims, this->seed) {
  _display = map_info->_show_graphics;
  // call rrt constructor
  _rrt.set_root(MotionPlanning::_pt_start);
}

KDPoint RRTPlan::_GenerateRandPoint(void) {
  unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
  std::default_random_engine generator(seed);
  std::uniform_int_distribution<int> dis_s(0, 9);
  if (dis_s(generator) < 2) {
    return MotionPlanning::_pt_end;
    std::uniform_int_distribution<int> dis_s;
  } else {
    std::uniform_int_distribution<int> dis_x(
        int(MotionPlanning::_map_info->min_x),
        int(MotionPlanning::_map_info->max_x));
    std::uniform_int_distribution<int> dis_y(
        int(MotionPlanning::_map_info->min_y),
        int(MotionPlanning::_map_info->max_y));
    while (true) {
      int x = dis_x(generator);
      int y = dis_y(generator);
      KDPoint p = {double(x), double(y)};
      if (!MotionPlanning::_map_info->Collision(p)) {
        return p;
      }
    }
  }
}

std::vector<KDPoint> RRTPlan::_ReconstrucPath(void) {
  std::vector<KDPoint> path;
  KDPoint p = MotionPlanning::_pt_end;
  while (p != MotionPlanning::_pt_start) {
    path.push_back(p);
    p = _rrt.GetParent(p);
  }
  path.push_back(p);
  return path;
}
// this is the standard implementation of rrt, which is left as default for a
// comparison with our implementation. We implemented our ideas for rrt* and
// rrt* dubins
std::tuple<std::vector<KDPoint>, double> RRTPlan::run(void) {
  int iter = 0;
  while (true) {
    KDPoint q_rand = _GenerateRandPoint();
    KDPoint q_near = _rrt.SearchNearestVertex(q_rand, iter);
    KDPoint q_new = _rrt.CalcNewPoint(q_near, q_rand);
    if (MotionPlanning::_map_info->Collision(q_new)) {
      continue;
    }

    _rrt.Add(q_new, q_near);
    if (MotionPlanning::_display) {
      MotionPlanning::_map_info->set_rrt(_rrt, 0, q_rand);
    }

    if (Distance(q_new, MotionPlanning::_pt_end) < 1.0) {
      if (q_new != MotionPlanning::_pt_end) {
        _rrt.Add(MotionPlanning::_pt_end, q_new);
      }
      std::tuple<std::vector<KDPoint>, double> final_path_cost =
          std::make_tuple(_ReconstrucPath(),
                          _rrt.Cost((MotionPlanning::_pt_end), true));
      return final_path_cost;
    }
    iter++;
  }
}