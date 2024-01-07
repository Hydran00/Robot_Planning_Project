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

using namespace std;

typedef std::tuple<std::vector<double>, std::vector<double>> Paths;

void print_path_on_file(Paths path)
{
    std::string file_path = ament_index_cpp::get_package_share_directory("planner") +
                            "/data/dubins_path.txt";
    std::ofstream fout;
    fout.open(file_path, std::ios_base::app);
    for (size_t i = 0; i < std::get<0>(path).size(); i++)
    {
        fout << std::get<0>(path)[i] << ", " << std::get<1>(path)[i] << std::endl;
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto m = std::make_shared<MapInfo>();

    sleep(1.5);
    RCLCPP_INFO(m->get_logger(), "Waiting for obstacles, borders and gates...");
    while (!m->obstacles_received_ || !m->borders_received_ || !m->gates_received_)
    {
        rclcpp::spin_some(m);
    }
    // string wkt = "";
    // Linestring p;
    // boost::geometry::read_wkt(wkt, p);
    // std::cout << "WITHIN: " << (boost::geometry::within(p, m->_map) ? "YES" : "NO") << std::endl;

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

    std::string file_path = ament_index_cpp::get_package_share_directory("planner") + "/data/dubins_path.txt";
    std::reverse(path.begin(), path.end());
    for (auto p : path)
    {
        std::cout << "( X: " << p[0] << ", Y: " << p[1] << " YAW: " << p[2] << " )" << std::endl;
    }

    Paths full_dubins_path;

    auto dubins_section = get_dubins_best_path_and_cost(m->pt_start, path[0], plan._radius, 0.1);

    // Insert first section
    std::get<0>(full_dubins_path).insert(std::get<0>(full_dubins_path).end(), std::get<0>(dubins_section).begin(), std::get<0>(dubins_section).end());
    std::get<1>(full_dubins_path).insert(std::get<1>(full_dubins_path).end(), std::get<1>(dubins_section).begin(), std::get<1>(dubins_section).end());
    // Insert all other sections
    for (size_t i = 1; i < path.size(); i++)
    {
        dubins_section = get_dubins_best_path_and_cost(path[i - 1], path[i], plan._radius, 0.1);
        std::get<0>(full_dubins_path).insert(std::get<0>(full_dubins_path).end(), std::get<0>(dubins_section).begin(), std::get<0>(dubins_section).end());
        std::get<1>(full_dubins_path).insert(std::get<1>(full_dubins_path).end(), std::get<1>(dubins_section).begin(), std::get<1>(dubins_section).end());
    }
    std::remove(file_path.c_str());
    print_path_on_file(full_dubins_path);
    Linestring l;
    for (size_t i = 0; i < std::get<0>(full_dubins_path).size(); ++i)
    {
        std::cout << std::get<0>(full_dubins_path)[i] << ", " << std::get<1>(full_dubins_path)[i] << std::endl;
        l.push_back(point_xy(std::get<0>(full_dubins_path)[i], std::get<1>(full_dubins_path)[i]));
    }
    // print X Y YAW

    cout << "IS PATH VALID?: " << (boost::geometry::within(l, m->_map) ? "YES" : "NO") << endl;

    // print path
    if (!path.empty())
    {
        m->set_dubins_path(full_dubins_path);
    }
    auto time_end = rclcpp::Clock().now();
    auto time_diff = time_end - time_start;
    cout << "Planning time: " << time_diff.seconds() << " seconds" << endl;
    rclcpp::shutdown();
    cout << "Done!" << endl;
    return 0;
}