#ifndef __RRT_STAR_PLAN__
#define __RRT_STAR_PLAN__

#include <vector>
#include <random>
#include <chrono>
#include "utils/motionplanning.hpp"
#include "utils/kdtree.hpp"
#include "utils/rrt.hpp"

class RRTStarPlan : public MotionPlanning
{
private:
    RRT _rrt;
    KDPoint _GenerateRandPoint(void);
    std::vector<KDPoint> _ReconstrucPath(void);
public:
    RRTStarPlan(std::shared_ptr<MapInfo> &map_info);
    std::vector<KDPoint> run(void);
};

#endif // !__RRT_STAR_PLAN__