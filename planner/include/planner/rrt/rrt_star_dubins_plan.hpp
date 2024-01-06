#ifndef __RRT_STAR_DUBINS_PLAN__
#define __RRT_STAR_DUBINS_PLAN__

#include <vector>
#include <random>
#include <chrono>
#include "utils/motionplanning.hpp"
#include "utils/kdtree.hpp"
#include "utils/rrt.hpp"

class RRTStarDubinsPlan : public MotionPlanning
{
private:
    RRT _rrt;
    KDPoint _GenerateRandPoint(void);
    std::vector<KDPoint> _ReconstrucPath(void);
public:
    double _radius;
    RRTStarDubinsPlan(std::shared_ptr<MapInfo> &map_info, double radius);
    std::vector<KDPoint> run(void);
    std::vector<std::vector<double>> final_path;
};

#endif // !__RRT_STAR_DUBINS_PLAN__