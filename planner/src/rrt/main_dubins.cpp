#include <iostream>
#include <unistd.h>
#include <fstream>

#include "planner/rrt/utils/kdtree.hpp"
#include "planner/rrt/utils/rrt.hpp"
#include "planner/rrt/utils/map_info.hpp"
#include "planner/rrt/rrt_plan.hpp"
#include "planner/rrt/rrt_star_dubins_plan.hpp"
#include "planner/dubins/dubins.h"
#include "ament_index_cpp/get_package_share_directory.hpp"

void print_section_on_file(std::tuple<std::vector<std::vector<double>>, std::vector<std::vector<double>>> path)
{
    std::string file_path = ament_index_cpp::get_package_share_directory("planner") + "/data/dubins_path.txt";
    std::ofstream fout;
    fout.open(file_path, std::ios_base::app);

    for (size_t i = 0; i < std::get<0>(path).size(); ++i)
    {
        for (size_t j = 0; j < std::get<0>(path)[i].size(); ++j)
        {
            fout << std::get<0>(path)[i][j] << ", " << std::get<1>(path)[i][j] << std::endl;
        }
    }
    fout.close();
}
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
        sleep(0.1);
        std::cout << "Waiting for obstacles, borders and gates..." << std::endl;
        // sleep(1);
        // rclcpp::spin_some(m);
        // rclcpp::sleep_for(std::chrono::milliseconds(10));
    }
    KDPoint point = {0, 0, 0};
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
    RRTStarDubinsPlan plan(m, radius);
    path = plan.run();

    if (!path.empty())
    {
        m->set_path(path);
    }

    std::string file_path = ament_index_cpp::get_package_share_directory("planner") + "/data/dubins_path.txt";
    std::remove(file_path.c_str());
    // print path
    Linestring best_path;
    DubinsPath dubins_path(m->pt_start, path[0], plan._radius);
    auto paths = dubins_path.calc_paths();
    auto shortest_path = dubins_path.get_shortest_path();
    auto full_dubins_path = dubins_path.gen_path(m->pt_start, shortest_path, plan._radius, 0.1, true);
    // send to the file
    print_section_on_file(full_dubins_path);
    for (size_t i = 1; i < path.size(); i++)
    {
        // DubinsPath reconstructed_path(m->pt_start, m->pt_end, m->_radius);
        // best_path.push_back(point_xy(path[i][0], path[i][1]));
        // cout << "( X: " << path[i][0] << ", Y: " << path[i][1] << " YAW: " << path[i][2] << " )" << endl;

        // dubins_path = DubinsPath(path[i-1], path[i], plan._radius);
        // paths = dubins_path.calc_paths();
        // shortest_path = dubins_path.get_shortest_path();
        // full_dubins_path = dubins_path.gen_path(m->pt_start, shortest_path, plan._radius, 0.1, true);
        // print_section_on_file(full_dubins_path);
        // cout << boost::geometry::wkt(best_path) << endl;
        // cout << "WITHIN: " << (boost::geometry::within(best_path, m->_map) ? "TRUE" : "FALSE") << endl;
        // cout << "########################################" << endl;
    }

    // print path
    for (auto p : best_path)
    {
    }
    auto time_end = rclcpp::Clock().now();
    auto time_diff = time_end - time_start;
    cout << "Planning time: " << time_diff.seconds() << " seconds" << endl;
    executor.cancel();
    // executor_thread.join();
    rclcpp::shutdown();
    cout << "Done!" << endl;
    return 0;
    // iterate inner rings
}