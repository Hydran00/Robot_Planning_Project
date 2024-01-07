#include "planner/rrt/utils/rrt_dubins.hpp"

void RRTDubins::set_root(KDPoint &p)
{
    _root.assign(p.begin(), p.end());
    _rrt.push_back(std::make_tuple(_root, 0, std::vector<std::vector<double>>()));
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

void RRTDubins::Add(KDPoint &q_new, KDPoint &q_near, Path &path)
{
    int i = std::find_if(
                _rrt.begin(), _rrt.end(),
                [&](std::tuple<KDPoint, int, Path> &tuple)
                {
                    return (std::get<0>(tuple) == q_near);
                }) -
            _rrt.begin();
    _rrt.push_back(std::make_tuple(q_new, i, path));
}

std::tuple<KDPoint, int, Path> RRTDubins::GetParent(KDPoint &p)
{
    auto it = std::find_if(
        _rrt.begin(), _rrt.end(),
        [&](std::tuple<KDPoint, int, Path> &tuple)
        {
            return (std::get<0>(tuple) == p);
        });
    // return _rrt[it->second].first;
    return _rrt[std::get<1>(*it)];
}

// def cost(self, q):
//     q_now = q
//     c = 0
//     while(q_now != self._root):
//         q_now, path, _ = self.get_info(q_now)
//         for p in path:
//             c += p[1] if p[0] == 's' else p[1] * self._r
//     return c

double RRTDubins::Cost(KDPoint p, double radius)
{
    double cost = 0.0;
    KDPoint q = p;
    while (q != _root)
    {
        std::tuple<KDPoint, int, Path> f = GetParent(q);
        for (auto path : std::get<2>(f))
        {
            cost += (path[0] == 's' ? path[1] : path[1] * radius);
        }
        q = std::get<0>(f);
    }
    return cost;
}

void RRTDubins::DubinsRewire(KDPoint &p, double r, std::function<bool(std::tuple<std::vector<double>, std::vector<double>> &path)> DubinsCollision, double dubins_radius)
{
    std::vector<KDPoint> nears;
    auto it_p = std::find_if(
        _rrt.begin(), _rrt.end(),
        [&](std::tuple<KDPoint, int, Path> &tuple)
        {
            return (std::get<0>(tuple) == p);
        });
    // fill nears
    std::for_each(
        _rrt.begin(), _rrt.end(),
        [&](std::tuple<KDPoint, int, Path> &tuple)
        {
            if ((std::get<0>(tuple) != p) && (Distance(std::get<0>(tuple), p) < r))
            {
                nears.push_back(std::get<0>(tuple));
            }
        });
    for (auto pt : nears)
    {
        auto dubins_best_path =
            get_dubins_best_path_and_cost(p, pt, dubins_radius, 0.1);
        
        // Check collision
        std::vector<double> x = std::get<0>(dubins_best_path);
        std::vector<double> y = std::get<1>(dubins_best_path);
        std::tuple<std::vector<double>, std::vector<double>> path = std::make_tuple(x,y);
        if(DubinsCollision(path))
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
        if (Cost(pt, dubins_radius) + distance < Cost(p, dubins_radius))
        {
            int idx = std::find_if(
                          _rrt.begin(), _rrt.end(),
                          [&](std::tuple<KDPoint, int, Path> &tuple)
                          {
                              return (std::get<0>(tuple) == pt);
                          }) -
                      _rrt.begin();
            // it_p->second = idx;
            std::get<1>(*it_p) = idx;
        }
    }
    for (auto pt : nears)
    {
        if (Cost(p,dubins_radius) + Distance(pt, p) < Cost(pt,dubins_radius))
        {
            auto it_pt = std::find_if(
                _rrt.begin(), _rrt.end(),
                [&](std::tuple<KDPoint, int, Path> &tuple)
                {
                    return (std::get<0>(tuple) == pt);
                }
            );
            // it_pt->second = int(it_p - _rrt.begin());
            std::get<1>(*it_pt) = int(it_p - _rrt.begin());
        }
    }
}
