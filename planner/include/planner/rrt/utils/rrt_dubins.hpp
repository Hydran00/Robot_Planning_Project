#ifndef __RRT_DUBINS__
#define __RRT_DUBINS__

#include <vector>
#include <tuple>
#include <utility>
#include <cmath>
#include <functional>
#include <algorithm>
#include "kdtree.hpp"
#include "planner/dubins/dubins.h"

class RRTDubins
{
private:
    KDPoint _root;
    std::vector<std::pair<KDPoint, int>> _rrt;
    const double branch_lenght = 0.2;

public:
    class iterator
    {
    private:
        const std::vector<std::pair<KDPoint, int>>::iterator _it_begin;
        int _pos;

    public:
        iterator(const std::vector<std::pair<KDPoint, int>>::iterator begin, int pos) : _it_begin(begin), _pos(pos) {}
        inline bool operator!=(iterator &it)
        {
            return (_pos != it._pos);
        }
        inline KDPoint operator*()
        {
            return (_it_begin + _pos)->first;
        }
        inline iterator &operator++()
        {
            ++_pos;
            return *this;
        }
    };
    // branch step length

    RRTDubins() {}
    void set_root(KDPoint &p);
    KDPoint SearchNearestVertex(KDPoint &q_rand);
    KDPoint CalcNewPoint(KDPoint &q_near, KDPoint &q_rand);
    void Add(KDPoint &q_new, KDPoint &q_near);
    KDPoint GetParent(KDPoint &p);
    double Cost(KDPoint &p);
    void DubinsRewire(KDPoint &p, double r, std::function<bool (std::tuple<std::vector<double>,std::vector<double>> &path)> DubinsCollision, double dubins_radius);
    inline int size(void)
    {
        return _rrt.size();
    }
    inline iterator begin()
    {
        return iterator(_rrt.begin(), 0);
    }
    inline iterator end()
    {
        return iterator(_rrt.begin(), size());
    }
};

#endif // !__RRT_DUBINS__