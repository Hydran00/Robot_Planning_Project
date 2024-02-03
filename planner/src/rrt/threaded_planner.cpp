#include "planner/rrt/threaded_planner.hpp"

// template <class PlannerType>
// ThreadedPlanner<PlannerType>::ThreadedPlanner(int thread_number,
//                                 const std::shared_ptr<MapInfo> map_info)
// {
//   this->thread_number = thread_number;
//   this->map_info = map_info;
// }

// Wrapper function for planner
// template <class PlannerType>
// void ThreadedPlanner<PlannerType>::_run_wrapper()
// {
//   PlannerType planner(this->map_info);
//   std::tuple<std::vector<KDPoint>, double> path_cost = planner.run();
//   // serialize to avoid concurrency issues
//   std::lock_guard<std::mutex> lock(vectorMutex);
//   path_list.push_back(std::get<0>(path_cost));
//   cost_list.push_back(std::get<1>(path_cost));
// }

// Choose the shortest path
// template <class PlannerType>
// std::vector<KDPoint> ThreadedPlanner<PlannerType>::_choose_path()
// {
//   int idx = std::min_element(cost_list.begin(), cost_list.end()) -
//             cost_list.begin();
//   for (size_t i = 0; i < path_list.size(); i++)
//   {
//     std::cout << "Path " << i << " : " << cost_list[i] << std::endl;
//   }
//   bool are_all_inf = true;
//   for (size_t i = 0; i < path_list.size(); i++)
//   {
//     if (cost_list[i] != std::numeric_limits<double>::infinity())
//     {
//       are_all_inf = false;
//       break;
//     }
//   }
//   if (are_all_inf)
//   {
//     std::cout << "All paths are inf" << std::endl;
//     return path_list[0];
//   }
//   std::cout << "Returning best path-> index: " << idx << std::endl;
//   return path_list[idx];
// }

// template <class PlannerType>
// std::vector<KDPoint> ThreadedPlanner<PlannerType>::execute_plans()
// {
//   std::vector<std::thread> threads;
//
//   for (int i = 0; i < thread_number; i++)
//   {
//     std::cout << "Creating thread " << i << std::endl;
//     std::thread t(&ThreadedPlanner::_run_wrapper, this);
//     threads.push_back(std::thread(std::move(t)));
//   }
//
//   for (auto &thread : threads)
//   {
//     thread.join();
//     std::cout << "Joined thread" << std::endl;
//   }
//
//   return _choose_path();
// }
