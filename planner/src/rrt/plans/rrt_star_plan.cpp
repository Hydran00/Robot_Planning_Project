#include "planner/rrt/rrt_star_plan.hpp"

RRTStarPlan::RRTStarPlan(std::shared_ptr<MapInfo> &map_info)
    : MotionPlanning(map_info) {
  _display = map_info->_show_graphics;
  _rrt.set_root(MotionPlanning::_pt_start);
}

KDPoint RRTStarPlan::_GenerateRandPoint(void) {
  unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
  std::default_random_engine generator(seed);
  std::uniform_int_distribution<int> dis_s(0, 9);
  // Epsilon greedy sampling
  if (dis_s(generator) < 2) {
    return MotionPlanning::_pt_end;
    // std::uniform_int_distribution<int> dis_s; ???????
  } else {
    // sample from the square embedding the map
    std::uniform_real_distribution<> dis_x((MotionPlanning::_map_info->min_x),
                                           (MotionPlanning::_map_info->max_x));
    std::uniform_real_distribution<> dis_y((MotionPlanning::_map_info->min_y),
                                           (MotionPlanning::_map_info->max_y));
    while (true) {
      // TODO: check if double sampling works
      double x = dis_x(generator);
      double y = dis_y(generator);
      KDPoint p = {double(x), double(y)};
      if (!MotionPlanning::_map_info->Collision(p)) {
        return p;
      }
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
  return path;
}

std::vector<KDPoint> RRTStarPlan::run(void) {
  while (true) {
    KDPoint q_rand = _GenerateRandPoint();
    KDPoint q_near = _rrt.SearchNearestVertex(q_rand);
    KDPoint q_new = _rrt.CalcNewPoint(q_near, q_rand);
    // TODO check
    // if (MotionPlanning::_map_info->Collision(q_new))
    // {
    //     continue;
    // }
    if (MotionPlanning::_map_info->Collision(q_near, q_new)) {
      continue;
    }
    _rrt.Add(q_new, q_near);
    // TODO check radius -> was 5.0
    _rrt.Rewire(q_new, 3.0, [&](KDPoint &p1, KDPoint &p2) {
      return MotionPlanning::_map_info->Collision(p1, p2);
    });
    if (MotionPlanning::_display) {
      MotionPlanning::_map_info->set_rrt(_rrt, 0, q_rand);
    }

    // IDEA -> when we found a solution we sample from the path
    // to decrease the cost
    // TODO -> was 1
    if (Distance(q_new, MotionPlanning::_pt_end) < 0.1) {
      if (q_new != MotionPlanning::_pt_end) {
        _rrt.Add(MotionPlanning::_pt_end, q_new);
      }
      return _ReconstrucPath();
    }
  }
}