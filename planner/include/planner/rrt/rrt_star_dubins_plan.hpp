#ifndef __RRT_STAR_DUBINS_PLAN__
#define __RRT_STAR_DUBINS_PLAN__

#include <vector>
#include <random>
#include <chrono>
#include "utils/motionplanning.hpp"
#include "utils/kdtree.hpp"
#include "utils/rrt_dubins.hpp"

class RRTStarDubinsPlan : public MotionPlanning
{
private:
    RRTDubins _rrt;
    KDPoint _GenerateRandPoint(int iter);
    std::vector<KDPoint> _ReconstrucPath();
public:
    double _radius;
    // [( [x1a,x1b,x1c,...],[y1a,y1b,y1c,...),( [x2a,x2b,x2c,...],[y2a,y2b,y2c,...),...]
    std::vector<std::tuple<std::vector<double>,std::vector<double>>> _paths;
    RRTStarDubinsPlan(std::shared_ptr<MapInfo> &map_info);
    std::tuple<std::vector<KDPoint>,double> run(void);
};

#endif // !__RRT_STAR_DUBINS_PLAN__