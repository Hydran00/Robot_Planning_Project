#include "planner/rrt/utils/rrt.hpp"

void RRT::set_root(KDPoint &p)
{
    _root.assign(p.begin(), p.end());
    _rrt.push_back(std::make_pair(_root, 0));
}

KDPoint RRT::SearchNearestVertex(KDPoint &q_rand)
{
    std::vector<double> d;
    for (auto pair : _rrt)
    {
        d.push_back(Distance(pair.first, q_rand));
    }
    int i = std::min_element(d.begin(), d.end()) - d.begin();
    return _rrt[i].first;
}

KDPoint RRT::CalcNewPoint(KDPoint &q_near, KDPoint &q_rand)
{
    // TODO -> was 1 ,branch_lenght was 1
    if (Distance(q_near, q_rand) < 0.4)
    {
        return q_rand;
    }

    double angle = std::atan2(q_rand[1] - q_near[1], q_rand[0] - q_near[0]);
    double x_new = q_near[0] + branch_lenght * std::cos(angle);
    double y_new = q_near[1] + branch_lenght * std::sin(angle);
    KDPoint p = {x_new, y_new};
    return p;
}

void RRT::Add(KDPoint &q_new, KDPoint &q_near)
{
    int i = std::find_if(
        _rrt.begin(), _rrt.end(),
        [&](std::pair<KDPoint, int> &pair)
        {
            return (pair.first == q_near);
        }
    ) - _rrt.begin();
    _rrt.push_back(std::make_pair(q_new, i));
}

KDPoint RRT::GetParent(KDPoint &p)
{
    auto it = std::find_if(
        _rrt.begin(), _rrt.end(),
        [&](std::pair<KDPoint, int> &pair)
        {
            return (pair.first == p);
        }
    );
    return _rrt[it->second].first;
}

double RRT::Cost(KDPoint &point)
{
    double c = 0.0;
    KDPoint p = point;
    while (p != _root)
    {
        KDPoint f = GetParent(p);
        c += Distance(p, f);
        p = f;
    }
    return c;
}
    // def rewire(self, q_new, r, map_info):
    //     q_near = [q for q in self._rrt.keys() if distance(q_new, q) <= r and q_new != q]
    //     for q in q_near:
    //         dubins = DubinsPath(q, q_new, self._r)
    //         dubins.calc_paths()
    //         path, length = dubins.get_shortest_path()
    //         xs, ys = DubinsPath.gen_path(q, path, self._r, section=False)
    //         if map_info.is_collision(path=[xs, ys]):
    //             continue
    //         if self.cost(q) + length < self.cost(q_new):
    //             self._rrt[q_new] = [q, path, [xs, ys]]
    //     for q in q_near:
    //         if q == self._rrt[q_new][0]:
    //             continue
    //         dubins = DubinsPath(q_new, q, self._r)
    //         dubins.calc_paths()
    //         path, length = dubins.get_shortest_path()
    //         xs, ys = DubinsPath.gen_path(q_new, path, self._r, section=False)
    //         if map_info.is_collision(path=[xs, ys]):
    //             continue
    //         if self.cost(q_new) + length < self.cost(q):
    //             self._rrt[q] = [q_new, path, [xs, ys]]

void RRT::Rewire(KDPoint &p, double r, std::function<bool (KDPoint &p1, KDPoint &p2)> Collision)
{
    std::vector<KDPoint> nears;
    auto it_p = std::find_if(
        _rrt.begin(), _rrt.end(),
        [&](std::pair<KDPoint, int> &pair)
        {
            return (pair.first == p);
        }
    );
    std::for_each(
        _rrt.begin(), _rrt.end(),
        [&](std::pair<KDPoint, int> &pair)
        {
            if ((pair.first != p) && (Distance(pair.first, p) < r) && (!Collision(pair.first, p)))
            {
                nears.push_back(pair.first);
            }
        }
    );
    for (auto pt : nears)
    {
        if (Cost(pt) + Distance(pt, p) < Cost(p))
        {
            int idx = std::find_if(
                _rrt.begin(), _rrt.end(),
                [&](std::pair<KDPoint, int> &pair)
                {
                    return (pair.first == pt);
                }
            ) - _rrt.begin();
            it_p->second = idx;
        }
    }
    for (auto pt : nears)
    {
        if (Cost(p) + Distance(pt, p) < Cost(pt))
        {
            auto it_pt = std::find_if(
                _rrt.begin(), _rrt.end(),
                [&](std::pair<KDPoint, int> &pair)
                {
                    return (pair.first == pt);
                }
            );
            it_pt->second = int(it_p - _rrt.begin());
        }
    }
}

void RRT::DubinsRewire(KDPoint &p, double r, std::function<bool (std::tuple<std::vector<double>,std::vector<double>> &path)> DubinsCollision, double dubins_radius)
{
    std::vector<KDPoint> nears;
    auto it_p = std::find_if(
        _rrt.begin(), _rrt.end(),
        [&](std::pair<KDPoint, int> &pair)
        {
            return (pair.first == p);
        }
    );
    // fill nears
    std::for_each(
        _rrt.begin(), _rrt.end(),
        [&](std::pair<KDPoint, int> &pair)
        {
            if ((pair.first != p) && (Distance(pair.first, p) < r))
            {
                nears.push_back(pair.first);
            }
        }
    );
    for (auto pt : nears)
    {
        std::tuple<std::vector<double>, std::vector<double>> dubins_best_path =
            get_dubins_best_path(p, pt, dubins_radius, 0.1);
        
        // if (Cost(pt) + Distance(pt, p) < Cost(p))
        // {
        //     int idx = std::find_if(
        //         _rrt.begin(), _rrt.end(),
        //         [&](std::pair<KDPoint, int> &pair)
        //         {
        //             return (pair.first == pt);
        //         }
        //     ) - _rrt.begin();
        //     it_p->second = idx;
        // }

    }
    for (auto pt : nears)
    {
        if (Cost(p) + Distance(pt, p) < Cost(pt))
        {
            auto it_pt = std::find_if(
                _rrt.begin(), _rrt.end(),
                [&](std::pair<KDPoint, int> &pair)
                {
                    return (pair.first == pt);
                }
            );
            it_pt->second = int(it_p - _rrt.begin());
        }
    }
}

