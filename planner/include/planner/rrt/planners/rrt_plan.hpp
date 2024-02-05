#ifndef __RRT_PLAN__
#define __RRT_PLAN__

#include <chrono>
#include <random>
#include <vector>

#include "planner/rrt/utils/kdtree.hpp"
#include "planner/rrt/utils/motionplanning.hpp"
#include "planner/rrt/utils/rrt.hpp"

class RRTPlan : public MotionPlanning {
 private:
  RRT _rrt;
  KDPoint _GenerateRandPoint(void);
  std::vector<KDPoint> _ReconstrucPath(void);

 public:
  RRTPlan(std::shared_ptr<MapInfo> &map_info);
  std::tuple<std::vector<KDPoint>, double> run(void);
  std::default_random_engine generator;
};

#endif  // !__RRT_PLAN__