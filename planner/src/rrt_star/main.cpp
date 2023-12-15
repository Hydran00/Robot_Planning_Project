#include <iostream>
#include <unistd.h>
#include <fstream>

#include "planner/rrt_star/kdtree.hpp"
#include "planner/rrt_star/rrt.hpp"
#include "planner/rrt_star/rrt_plan.hpp"
#include "planner/rrt_star/rrt_star_plan.hpp"
#include "planner/rrt_star/map_info.hpp"


using namespace std;
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto m = std::make_shared<MapInfo>();

    // Create a MultiThreadedExecutor
    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

    // Add your node to the executor
    executor->add_node(m);

    // Spin the executor in a separate thread
    auto executor_thread = std::thread([&executor]()
                                       { executor->spin(); });
    while(!m->obstacles_received_ || !m->borders_received_ || !m->gates_received_)
    {
        sleep(0.01);
    }
    // m->set_boundary(60, 40);
    // std::vector<KDPoint> points;
    // for (int i = 0; i < 30; ++i)
    // {
    //     KDPoint p = {20, double(i)};
    //     points.push_back(p);
    // }

    // for (int i = 0; i < 30; ++i)
    // {
    //     KDPoint p = {40, 40 - double(i)};
    //     points.push_back(p);
    // }

    // m->set_obstacle(points);
    KDPoint point = {0,0};
    m->set_start(point);
    // point = {-0.8, -6.3};
    // m->set_end(point);

    if (m->_show_graphics)
        m->ShowMap();
    sleep(2.0);
    std::vector<KDPoint> path;

    RRTStarPlan plan(m);
    path = plan.run();

    if (!path.empty())
        m->set_path(path);

    // print path   
    for (auto p : path)
    {
        std::cout << p[0] << " " << p[1] << std::endl;
    }
    return 0;
}