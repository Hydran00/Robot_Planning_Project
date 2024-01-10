#ifndef __RRT_DUBINS__
#define __RRT_DUBINS__

#include <algorithm>
#include <cmath>
#include <functional>
#include <tuple>
#include <utility>
#include <vector>

#include "kdtree.hpp"
#include "planner/dubins/dubins.h"
// # include "planner/rrt/utils/map_info.hpp"

// real path is a vector in the form [ (x1,y1), (x2,y2), ...])]
typedef std::tuple<std::vector<double>, std::vector<double>> Path;
// symbolyc path is a vector like [ (lsl), (lrl), (rsl), ...]
typedef std::vector<std::vector<double>> SymbolicPath;

typedef std::vector<std::tuple<KDPoint, int, SymbolicPath, Path>> DubinsTree;
class RRTDubins
{
private:
  KDPoint _root;

public:
  DubinsTree _rrt;
  class iterator
  {
  private:
    const DubinsTree::iterator _it_begin;
    int _pos;

  public:
    iterator(const DubinsTree::iterator begin, int pos)
        : _it_begin(begin), _pos(pos) {}
    inline bool operator!=(iterator &it)
    {
      std::cout << "!= called: " << _pos << std::endl;
      return (_pos != it._pos);
    }
    inline KDPoint operator*()
    {
      // return (_it_begin + _pos)->first;
      std::cout << "* called: " << _pos << std::endl;
      return std::get<0>(*(_it_begin + _pos));
    }
    inline iterator &operator++()
    {
      ++_pos;
      std::cout << "pos called: " << _pos << std::endl;
      return *this;
    }
  };
  // branch step length

  RRTDubins() {}
  void set_root(KDPoint &p);
  KDPoint SearchNearestVertex(KDPoint &q_rand);
  // KDPoint CalcNewPoint(KDPoint &q_near, KDPoint &q_rand);
  void Add(KDPoint &q_new, KDPoint &q_near, SymbolicPath &sym_path, Path &path);
  double Cost(KDPoint p, double radius);
  void DubinsRewire(
      KDPoint &p, double r,
      std::function<
          bool(std::tuple<std::vector<double>, std::vector<double>> &path)>
          DubinsCollision,
      double dubins_radius);
  std::tuple<KDPoint, int, SymbolicPath, Path> GetParent(KDPoint &p);
  inline int size(void) { return _rrt.size(); }
  inline iterator begin() { return iterator(_rrt.begin(), 0); }
  inline iterator end() { return iterator(_rrt.begin(), size()); }
};

#endif // !__RRT_DUBINS__