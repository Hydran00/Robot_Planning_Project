#ifndef __RRT_PLAN__
#define __RRT_PLAN__

#include <vector>
#include <random>
#include <chrono>
#include "motionplanning.hpp"
#include "kdtree.hpp"
#include "rrt.hpp"

class RRTPlan : public MotionPlanning
{
private:
    RRT _rrt;
    KDPoint _GenerateRandPoint(void);
    std::vector<KDPoint> _ReconstrucPath(void);
public:
    RRTPlan(std::shared_ptr<MapInfo> &map_info);
    std::vector<KDPoint> run(void);
};

#endif // !__RRT_PLAN__