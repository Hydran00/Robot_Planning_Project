#include <iostream>
#include <thread>
#include <unistd.h>

#include "utils/kdtree.hpp"
#include "planner/rrt/rrt_star_dubins_plan.hpp"

class ThreadedPlanner {
private:
    std::vector<KDPoint> _choose_path();
    void _run_wrapper();
public:
    int thread_number;
    std::vector<std::vector<KDPoint>> path_list;
    // TODO: generalize to any planner
    RRTStarDubinsPlan planner;

    ThreadedPlanner(int thread_number, RRTStarDubinsPlan planner);
    std::Vector<KDPoint> execute_plans();
}


ThreadedPlanner::ThreadedPlanner(int trhead_number, RRTStarDubinsPlan planner) {
    this->thread_number = thread_number;
    this->planner = planner;
}

std::Vector<KDPoint> ThreadedPlanner::execute_plans() {
    std::vector<std::thread> threads;

    for (int i=0; i<thread_number; i++) {
        trheads.push_back(std::trhead(_execute_plan));
    }

    for(auto& thread in trheads) {
        trhead.join();
    }

    return _choose_path();
}

// Wrapper function for planner
void ThreadedPlanner::_run_wrapper() {
    path_list.push_back(planner.run());
}

// Choose the shortest path
std::vector<KDPoint> ThreadedPlanner::_choose_path() {
    
}

// <>