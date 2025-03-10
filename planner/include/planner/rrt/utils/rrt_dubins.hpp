#ifndef __RRT_DUBINS__
#define __RRT_DUBINS__

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

// symbolyc path is a vector like [ (lsl), (lrl), (rsl), ...]
typedef std::vector<std::vector<double>> SymbolicPath;

typedef std::vector<
    std::tuple<KDPoint, int, SymbolicPath, std::vector<KDPoint>>>
    DubinsTree;

class RRTDubins {
 private:
 public:
  KDPoint _root;
  DubinsTree _rrt;
  double _direct_cost_old = 0.0;
  std::default_random_engine generator;

  RRTDubins(std::vector<std::tuple<KDPoint, double>> &victims, uint seed) {
    this->victims = victims;
    this->generator = std::default_random_engine(seed);
  }
  std::vector<std::tuple<KDPoint, double>> victims;

  // RRT Dubins
  void set_root(KDPoint &p);
  std::tuple<KDPoint, int, SymbolicPath, std::vector<KDPoint>>

  SearchNearestVertex(KDPoint &q_rand, double radius, int iter);
  std::tuple<KDPoint, int, SymbolicPath, std::vector<KDPoint>> Add(
      KDPoint &q_new,
      std::tuple<KDPoint, int, SymbolicPath, std::vector<KDPoint>> &q_near,
      SymbolicPath &sym_path, std::vector<KDPoint> &path);

  double Cost(
      std::tuple<KDPoint, int, SymbolicPath, std::vector<KDPoint>> &node,
      double radius, bool consider_victims);

  double GetPathLength(SymbolicPath &sym_path, double radius);
  std::vector<KDPoint> GetPointPath(KDPoint &p);

  // RRT* Dubins
  void Rewire(
      std::tuple<KDPoint, int, SymbolicPath, std::vector<KDPoint>> &q_new,
      double r, 
      std::function<bool(std::vector<KDPoint> &path)> Collision,
      double dubins_radius
  );

  std::tuple<KDPoint, int, SymbolicPath, std::vector<KDPoint>> GetParent(
      KDPoint &p);

  // RRT*-smart Dubins
  bool PathOptimisation(
      std::tuple<KDPoint, int, SymbolicPath, std::vector<KDPoint>> &current_node_,
      std::tuple<KDPoint, int, SymbolicPath, std::vector<KDPoint>> &node_end,
      std::function<bool(std::vector<KDPoint> &path)> Collision,
      double dubins_radius);

  // Tree iterator
  class iterator {
   private:
    const DubinsTree::iterator _it_begin;
    int _pos;

   public:
    iterator(const DubinsTree::iterator begin, int pos)
        : _it_begin(begin), _pos(pos) {}
    inline bool operator!=(iterator &it) {
      return (_pos != it._pos);
    }
    inline KDPoint operator*() {
      // return (_it_begin + _pos)->first;
      return std::get<0>(*(_it_begin + _pos));
    }
    inline iterator &operator++() {
      ++_pos;
      return *this;
    }
  };

  inline int size(void) { return _rrt.size(); }
  inline iterator begin() { return iterator(_rrt.begin(), 0); }
  inline iterator end() { return iterator(_rrt.begin(), size()); }
};

#endif  // !__RRT_DUBINS__
