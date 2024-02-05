#ifndef __RRT__
#define __RRT__

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <random>
#include <tuple>
#include <utility>
#include <vector>

#include "kdtree.hpp"
#include "planner/dubins/dubins.h"
class RRT {
 private:
  // branch step length
  const double branch_lenght = 1.5;

 public:
  std::vector<std::pair<KDPoint, int>> _rrt;
  KDPoint _root;
  std::default_random_engine generator;
  RRT(std::vector<std::tuple<KDPoint, double>> &victims, uint seed) {
    this->victims = victims;
    
    this->generator = std::default_random_engine(seed);
  }
  // RRT(){};
  std::vector<std::tuple<KDPoint, double>> victims;
  void set_root(KDPoint &p);
  KDPoint SearchNearestVertex(KDPoint &q_rand, int iter);
  KDPoint CalcNewPoint(KDPoint &q_near, KDPoint &q_rand);
  void Add(KDPoint &q_new, KDPoint &q_near);
  KDPoint GetParent(KDPoint &p);
  double Cost(KDPoint &p, bool consider_victims);
  void Rewire(KDPoint &p, double r,
              std::function<bool(std::vector<KDPoint> &branch)> Collision);
  bool PathOptimisation(
      KDPoint &current_node_, KDPoint &node_end,
      std::function<bool(std::vector<KDPoint> &path)> Collision);



  class iterator {
   private:
    const std::vector<std::pair<KDPoint, int>>::iterator _it_begin;
    int _pos;

   public:
    iterator(const std::vector<std::pair<KDPoint, int>>::iterator begin,
             int pos)
        : _it_begin(begin), _pos(pos) {}
    inline bool operator!=(iterator &it) { return (_pos != it._pos); }
    inline KDPoint operator*() { return (_it_begin + _pos)->first; }
    inline iterator &operator++() {
      ++_pos;
      return *this;
    }
  };
  inline int size(void) { return _rrt.size(); }
  inline iterator begin() { return iterator(_rrt.begin(), 0); }
  inline iterator end() { return iterator(_rrt.begin(), size()); }

};

#endif  // !__RRT__