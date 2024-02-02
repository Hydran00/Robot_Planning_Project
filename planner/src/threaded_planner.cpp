#include <iostream>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <chrono>
#include <unistd.h>
#include <signal.h>

#include "planner/rrt/rrt_star_dubins_plan.hpp"
#include "utils/kdtree.hpp"

class ThreadedPlanner {
 private:
  std::vector<KDPoint> _choose_path();
  void _run_wrapper();
  std::mutex mtx;
  std::condition_variable cv;

 public:
  int thread_number;
  std::vector<std::vector<KDPoint>> path_list;
  // TODO: generalize to any planner (Use template)
  RRTStarDubinsPlan planner;

  ThreadedPlanner(int thread_number, RRTStarDubinsPlan planner);
  std::Vector<KDPoint> execute_plans();
}

ThreadedPlanner::ThreadedPlanner(int thread_number, RRTStarDubinsPlan planner) {
  this.thread_number = thread_number;
  this.planner = planner;
}

std::vector<KDPoint> ThreadedPlanner::execute_plans() {
  std::vector<std::thread> threads;

  for (int i = 0; i < thread_number; i++) {
    threads.push_back(std::thread(_run_wrapper));
  }

  // TODO: create timers to stop threads when the time is over
  for (int i = 0; i < thread_number; i++) {
    std::unique_lock<std::mutex> lock(mtx);
    cv.wait_for(lock, std::chrono::seconds(t));
    if (/*check if thread is still running*/) {
        pthread_t thread_id = threads[i].native_handle();
        pthread_kill(thread_id, SIGUSR1);
    }
  }

  cv.notify_all();

  for (auto& thread in threads) {
    thread.join();
  }

  return _choose_path();
}

// Wrapper function for planner
void ThreadedPlanner::_run_wrapper() {
  std::tuple<std::vector<KDPoint, double>> path_cost = planner.run();
  // TODO -> serialize to avoid concurrency issues
  path_list.push_back(std::get<0>(path_cost));
}

// Choose the shortest path
std::vector<KDPoint> ThreadedPlanner::_choose_path() {}

// <>