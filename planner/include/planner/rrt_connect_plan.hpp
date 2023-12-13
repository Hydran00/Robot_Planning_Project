#ifndef __RRT_CONNECT_PLAN__
#define __RRT_CONNECT_PLAN__

#include <vector>
#include <random>
#include <chrono>
#include "motionplanning.hpp"
#include "kdtree.hpp"
#include "rrt.hpp"

class RRTConnectPlan : public MotionPlanning
{
private:
    RRT _rrt_start;
    RRT _rrt_end;
    KDPoint _GenerateRandPoint(void);
    std::vector<KDPoint> _ReconstrucPath(RRT &rrt, KDPoint &start, KDPoint &end);
    std::vector<KDPoint> _ReconstrucPath(KDPoint &link);
public:
    RRTConnectPlan(std::shared_ptr<MapInfo> &map_info, bool display);
    std::vector<KDPoint> run(void);
};

#endif // !__RRT_CONNECT_PLAN__