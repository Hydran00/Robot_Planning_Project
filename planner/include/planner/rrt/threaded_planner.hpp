#ifndef __THREADED_PLANNER__
#define __THREADED_PLANNER__

#include <signal.h>
#include <unistd.h>

#include <chrono>
#include <condition_variable>
#include <iostream>
#include <mutex>
#include <thread>

#include "planner/rrt/rrt_star_dubins_plan.hpp"
#include "planner/rrt/utils/kdtree.hpp"

template <class PlannerType>
class ThreadedPlanner
{
private:
  std::vector<KDPoint> _choose_path();
  void _run_wrapper();
  int thread_number;
  std::vector<std::vector<KDPoint>> path_list;
  std::vector<double> cost_list;
  // TODO: generalize to any planner (Use template)
  std::shared_ptr<MapInfo> map_info;
  std::mutex vectorMutex;

public:
  ThreadedPlanner(int thread_number, const std::shared_ptr<MapInfo> map_info);
  std::vector<KDPoint> execute_plans();
};

#endif // __THREADED_PLANNER__
