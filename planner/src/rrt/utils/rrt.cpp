#include "planner/rrt/utils/rrt.hpp"

#include <unistd.h>

void RRT::set_root(KDPoint &p) {
  _root.assign(p.begin(), p.end());
  _rrt.push_back(std::make_pair(_root, 0));
}

KDPoint RRT::SearchNearestVertex(KDPoint &q_rand) {
  std::vector<double> d;

  for (auto pair : _rrt) {
    d.push_back(Distance(pair.first, q_rand) + 0.90 * Cost(pair.first, true));
  }
  int i = std::min_element(d.begin(), d.end()) - d.begin();
  return _rrt[i].first;
}

KDPoint RRT::CalcNewPoint(KDPoint &q_near, KDPoint &q_rand) {
  if (Distance(q_near, q_rand) < branch_lenght) {
    return q_rand;
  }

  double angle = std::atan2(q_rand[1] - q_near[1], q_rand[0] - q_near[0]);
  double x_new = q_near[0] + branch_lenght * std::cos(angle);
  double y_new = q_near[1] + branch_lenght * std::sin(angle);
  KDPoint p = {x_new, y_new};
  return p;
}

void RRT::Add(KDPoint &q_new, KDPoint &q_near) {
  int i = std::find_if(_rrt.begin(), _rrt.end(),
                       [&](std::pair<KDPoint, int> &pair) {
                         return (pair.first == q_near);
                       }) -
          _rrt.begin();
  _rrt.push_back(std::make_pair(q_new, i));
}

KDPoint RRT::GetParent(KDPoint &p) {
  auto it = std::find_if(
      _rrt.begin(), _rrt.end(),
      [&](std::pair<KDPoint, int> &pair) { return (pair.first == p); });
  return _rrt[it->second].first;
}

double RRT::Cost(KDPoint &point, bool consider_victims) {
  std::vector<std::tuple<KDPoint, double>> victims_list = victims;
  KDPoint p = point;
  double cost = 0.0;
  int i = 0;
  while (p != _root) {
    // checks if p is a victim
    if (consider_victims) {
      auto it = std::find_if(victims_list.begin(), victims_list.end(),
                             [&](std::tuple<KDPoint, double> &victim) {
                               return (std::get<0>(victim) == p);
                             });
      if (it != victims_list.end()) {
        cost -= std::get<1>(*it);
        victims_list.erase(it);
      }
    }
    KDPoint f = GetParent(p);
    cost += Distance(p, f);
    p = f;
    i++;
  }
  return cost;
}

void RRT::Rewire(KDPoint &p, double r,
                 std::function<bool(std::vector<KDPoint> &branch)> Collision) {
  std::vector<KDPoint> nears;
  auto it_p = std::find_if(
      _rrt.begin(), _rrt.end(),
      [&](std::pair<KDPoint, int> &pair) { return (pair.first == p); });
  std::for_each(_rrt.begin(), _rrt.end(), [&](std::pair<KDPoint, int> &pair) {
    std::vector<KDPoint> branch = {pair.first, p};
    if ((pair.first != p) && (Distance(pair.first, p) < r) &&
        (!Collision(branch))) {
      nears.push_back(pair.first);
    }
  });
  for (auto pt : nears) {
    if (Cost(pt, true) + Distance(pt, p) < Cost(p, true)) {
      int idx = std::find_if(_rrt.begin(), _rrt.end(),
                             [&](std::pair<KDPoint, int> &pair) {
                               return (pair.first == pt);
                             }) -
                _rrt.begin();
      it_p->second = idx;
    }
  }
  // for (auto pt : nears) {
  //   if (Cost(p) + Distance(pt, p) < Cost(pt)) {
  //     auto it_pt = std::find_if(
  //         _rrt.begin(), _rrt.end(),
  //         [&](std::pair<KDPoint, int> &pair) { return (pair.first == pt);
  //         });
  //     it_pt->second = int(it_p - _rrt.begin());
  //   }
  // }
}
