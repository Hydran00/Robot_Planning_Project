#include <iostream>
#include <unistd.h>
#include <fstream>
#include "planner/kdtree.hpp"
#include "planner/rrt.hpp"
#include "planner/rrt_plan.hpp"
#include "planner/rrt_star_plan.hpp"
#include "planner/map_info.hpp"
using namespace std;
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto m = std::make_shared<MapInfo>();
    m->set_boundary(60, 40);
    std::vector<KDPoint> points;
    for (int i = 0; i < 30; ++i)
    {
        KDPoint p = {20, double(i)};
        points.push_back(p);
    }

    for (int i = 0; i < 30; ++i)
    {
        KDPoint p = {40, 40 - double(i)};
        points.push_back(p);
    }

    m->set_obstacle(points);
    KDPoint point = {10, 10};
    m->set_start(point);
    point = {50, 30};
    m->set_end(point);

    m->ShowMap();

    sleep(1);
    std::vector<KDPoint> path;

    RRTStarPlan plan(m, true);
    path = plan.run();

    if (!path.empty())
        m->set_path(path);

    return 0;
}