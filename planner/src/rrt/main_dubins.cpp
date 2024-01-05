#include <iostream>
#include <unistd.h>
// #include <fstream>

#include "planner/rrt/utils/kdtree.hpp"
#include "planner/rrt/utils/rrt.hpp"
#include "planner/rrt/utils/map_info.hpp"
#include "planner/rrt/rrt_plan.hpp"
#include "planner/rrt/rrt_star_dubins_plan.hpp"
#include "planner/dubins/dubins.h"

using namespace std;
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto m = std::make_shared<MapInfo>();

    // Create a MultiThreadedExecutor
    rclcpp::executors::SingleThreadedExecutor executor;

    // Add your node to the executor
    executor.add_node(m);

    // // Spin the executor in a separate thread
    std::thread([&executor]()
                { executor.spin(); })
        .detach();
    RCLCPP_INFO(m->get_logger(), "Waiting for obstacles, borders and gates...");
    while (!m->obstacles_received_ || !m->borders_received_ || !m->gates_received_)
    {
        // sleep(1);
        // rclcpp::spin_some(m);
        rclcpp::sleep_for(std::chrono::milliseconds(10));
    }
    KDPoint point = {-0.7, -1.15, 0.4};
    while (m->Collision(point))
    {
        point[0] += 0.1;
    }
    m->set_start(point);

    if (m->_show_graphics)
    {
        m->ShowMap();
    }
    rclcpp::sleep_for(std::chrono::seconds(1));
    std::vector<KDPoint> path;

    // Monitor execution time
    auto time_start = rclcpp::Clock().now();
    double radius = 0.5;
    RRTStarDubinsPlan plan(m,radius);
    path = plan.run();

    if (!path.empty())
    {
        m->set_path(path);
    }

    // print path
    int z = 0;
    Linestring best_path;
    for (auto p : path)
    {
        best_path.push_back(point_xy(p[0], p[1]));
        cout << boost::geometry::wkt(best_path) << endl;
        z++;
        cout << "WITHIN: " << (boost::geometry::within(best_path, m->_map) ? "TRUE" : "FALSE") << endl;
        cout << "########################################" << endl;
    }
    auto time_end = rclcpp::Clock().now();
    auto time_diff = time_end - time_start;
    cout << "Planning time: " << time_diff.seconds() << " seconds" << endl;
    executor.cancel();
    executor_thread.join();
    rclcpp::shutdown();
    cout << "Done!" << endl;
    return 0;
    // iterate inner rings
}