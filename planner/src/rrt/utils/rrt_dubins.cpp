#include "planner/rrt/utils/rrt_dubins.hpp"

void RRTDubins::set_root(KDPoint &p)
{
  _root.assign(p.begin(), p.end());
  _rrt.push_back(std::make_tuple(
      _root, 0, std::vector<std::vector<double>>(),
      std::make_tuple(std::vector<double>(), std::vector<double>())));
}

KDPoint RRTDubins::SearchNearestVertex(KDPoint &q_rand)
{
  std::vector<double> d;
  for (auto tuple : _rrt)
  {
    // d.push_back(Distance(pair.first, q_rand));
    d.push_back(Distance(std::get<0>(tuple), q_rand));
  }
  int i = std::min_element(d.begin(), d.end()) - d.begin();
  return std::get<0>(_rrt[i]);
}

// KDPoint RRTDubins::CalcNewPoint(KDPoint &q_near, KDPoint &q_rand)
// {
//     // TODO -> was 1 ,branch_lenght was 1
//     if (Distance(q_near, q_rand) < 0.4)
//     {
//         return q_rand;
//     }

//     double angle = std::atan2(q_rand[1] - q_near[1], q_rand[0] - q_near[0]);
//     double x_new = q_near[0] + branch_lenght * std::cos(angle);
//     double y_new = q_near[1] + branch_lenght * std::sin(angle);
//     KDPoint p = {x_new, y_new};
//     return p;
// }

void RRTDubins::Add(KDPoint &q_new, KDPoint &q_near, SymbolicPath &sym_path,
                    Path &path)
{
  int i =
      std::find_if(_rrt.begin(), _rrt.end(),
                   [&](std::tuple<KDPoint, int, SymbolicPath, Path> &tuple)
                   {
                     return (std::get<0>(tuple) == q_near);
                   }) -
      _rrt.begin();
  _rrt.push_back(std::make_tuple(q_new, i, sym_path, path));
}

std::tuple<KDPoint, int, SymbolicPath, Path> RRTDubins::GetParent(KDPoint &p)
{
  auto it =
      std::find_if(_rrt.begin(), _rrt.end(),
                   [&](std::tuple<KDPoint, int, SymbolicPath, Path> &tuple)
                   {
                     return (std::get<0>(tuple) == p);
                   });
  // return _rrt[it->second].first;
  return _rrt[std::get<1>(*it)];
}

double RRTDubins::Cost(KDPoint p, double radius)
{
  double cost = 0.0;
  KDPoint q = p;
  while (q != _root)
  {
    std::tuple<KDPoint, int, SymbolicPath, Path> f = GetParent(q);
    for (auto path : std::get<2>(f))
    {
      cost += (path[0] == 's' ? path[1] : path[1] * radius);
    }
    q = std::get<0>(f);
  }
  return cost;
}

void RRTDubins::DubinsRewire(
    KDPoint &q_new, double r,
    std::function<
        bool(std::tuple<std::vector<double>, std::vector<double>> &path)>
        DubinsCollision,
    double dubins_radius)
{
  std::vector<KDPoint> nears;
  auto it_p =
      std::find_if(_rrt.begin(), _rrt.end(),
                   [&](std::tuple<KDPoint, int, SymbolicPath, Path> &tuple)
                   {
                     return (std::get<0>(tuple) == q_new);
                   });
  // fill nears
  std::for_each(_rrt.begin(), _rrt.end(),
                [&](std::tuple<KDPoint, int, SymbolicPath, Path> &tuple)
                {
                  if ((std::get<0>(tuple) != q_new) &&
                      (Distance(std::get<0>(tuple), q_new) < r))
                  {
                    nears.push_back(std::get<0>(tuple));
                  }
                });

  for (auto q : nears)
  {
    auto dubins_best_path =
        get_dubins_best_path_and_cost(q_new, q, dubins_radius, 0.1);

    // Check collision
    std::vector<double> x = std::get<0>(dubins_best_path);
    std::vector<double> y = std::get<1>(dubins_best_path);
    std::tuple<std::vector<double>, std::vector<double>> path =
        std::make_tuple(x, y);
    if (DubinsCollision(path))
    {
      continue;
    }

    // compute distance given the symbolic path
    double distance = 0.0;
    for (auto &p : std::get<3>(dubins_best_path))
    {
      if (p[0] == 's')
      {
        distance += p[1];
      }
      else
      {
        distance += p[1] * dubins_radius;
      }
    }
    // TODO CHANGE DISTANCE WITH LENGTH
    if (Cost(q, dubins_radius) + distance < Cost(q_new, dubins_radius))
    {
      int idx = std::find_if(
                    _rrt.begin(), _rrt.end(),
                    [&](std::tuple<KDPoint, int, SymbolicPath, Path> &tuple)
                    {
                      return (std::get<0>(tuple) == q);
                    }) -
                _rrt.begin();
      // it_p->second = idx;
      std::get<1>(*it_p) = idx;
      // Update symbolic path
      std::get<2>(*it_p) = std::get<3>(dubins_best_path);
      // Update path
      std::tuple<std::vector<double>, std::vector<double>> path =
          std::make_tuple(std::get<0>(dubins_best_path),
                          std::get<1>(dubins_best_path));
      std::get<3>(*it_p) = path;
    }
  }
  for (auto q : nears)
  {
    auto dubins_best_path =
        get_dubins_best_path_and_cost(q, q_new, dubins_radius, 0.1);

    // Check collision
    std::vector<double> x = std::get<0>(dubins_best_path);
    std::vector<double> y = std::get<1>(dubins_best_path);
    std::tuple<std::vector<double>, std::vector<double>> path =
        std::make_tuple(x, y);
    if (DubinsCollision(path))
    {
      continue;
    }

    double distance = 0.0;
    for (auto &p : std::get<3>(dubins_best_path))
    {
      if (p[0] == 's')
      {
        distance += p[1];
      }
      else
      {
        distance += p[1] * dubins_radius;
      }
    }
    if (Cost(q_new, dubins_radius) + distance < Cost(q, dubins_radius))
    {
      auto it_pt = std::find_if(
          _rrt.begin(), _rrt.end(),
          [&](std::tuple<KDPoint, int, SymbolicPath, Path> &tuple)
          {
            return (std::get<0>(tuple) == q);
          });
      // it_pt->second = int(it_p - _rrt.begin());
      std::get<1>(*it_pt) = int(it_p - _rrt.begin());

      // it_p->second = idx;
      // Update symbolic path
      std::get<2>(*it_p) = std::get<3>(dubins_best_path);
      // Update path
      std::tuple<std::vector<double>, std::vector<double>> path =
          std::make_tuple(std::get<0>(dubins_best_path),
                          std::get<1>(dubins_best_path));
      std::get<3>(*it_p) = path;
    }
  }
}
