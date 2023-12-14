#ifndef __MOTIONPLANNING__
#define __MOTIONPLANNING__

#include "map_info.hpp"
#include <vector>

class MotionPlanning
{
public:
    std::shared_ptr<MapInfo> &_map_info;
    bool _display;
    KDPoint _pt_start;
    KDPoint _pt_end;
    MotionPlanning(std::shared_ptr<MapInfo> &map_info) : _map_info(map_info)
    {
        _pt_start.assign(map_info->pt_start.begin(), map_info->pt_start.end());
        _pt_end.assign(map_info->pt_end.begin(), map_info->pt_end.end());
    }
    virtual std::vector<KDPoint> run(void) = 0;
};

#endif // !__MOTIONPLANNING__
