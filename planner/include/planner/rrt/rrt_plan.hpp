#ifndef __RRT_PLAN__
#define __RRT_PLAN__

#include <vector>
#include <random>
#include <chrono>
#include "utils/motionplanning.hpp"
#include "utils/kdtree.hpp"
#include "utils/rrt.hpp"

class RRTPlan : public MotionPlanning
{
private:
    RRT _rrt;
    KDPoint _GenerateRandPoint(void);
    std::vector<KDPoint> _ReconstrucPath(void);
public:
    RRTPlan(std::shared_ptr<MapInfo> &map_info, std::vector<std::tuple<KDPoint,double>> &victims);
    std::tuple<std::vector<KDPoint>,double> run(void);
};

#endif // !__RRT_PLAN__