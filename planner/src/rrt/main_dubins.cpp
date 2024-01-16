#include <unistd.h>

#include <fstream>
#include <future>
#include <iostream>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "planner/dubins/dubins.h"
#include "planner/rrt/rrt_plan.hpp"
#include "planner/rrt/rrt_star_dubins_plan.hpp"
#include "planner/rrt/utils/kdtree.hpp"
#include "planner/rrt/utils/map_info.hpp"
#include "planner/rrt/utils/rrt.hpp"

using namespace std;

typedef std::tuple<std::vector<double>, std::vector<double>> Path;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::string file_path =
      ament_index_cpp::get_package_share_directory("planner") +
      "/data/dubins_path.txt";
  std::remove(file_path.c_str());

  std::string map_path =
      ament_index_cpp::get_package_share_directory("planner") +
      "/data/map.txt";
  std::remove(file_path.c_str());

  auto m = std::make_shared<MapInfo>();

  auto future = std::async(std::launch::async, [&m]()
                           {
    while (rclcpp::ok() && (!m->obstacles_received_ || !m->borders_received_ ||
                            !m->gates_received_)) {
      rclcpp::sleep_for(std::chrono::milliseconds(1000));
      RCLCPP_INFO(m->get_logger(), "Waiting for map infos...");
    } });

  RCLCPP_INFO(m->get_logger(), "Waiting for obstacles, borders and gates...");
  // while (!m->obstacles_received_ || !m->borders_received_ ||
  //        !m->gates_received_) {
  //   rclcpp::spin_some(m->get_node_base_interface());
  //   rclcpp::sleep_for(std::chrono::milliseconds(100));
  // }
  rclcpp::spin_until_future_complete(m->get_node_base_interface(), future);
  RCLCPP_INFO(m->get_logger(), "\033[1;32m Map information received!\033[0m");

  KDPoint point = {0, 0, 0};
  while (m->Collision(point))
  {
    point[0] += 0.1;
  }
  std::cout << "Start is at " << point[0] << ", " << point[1] << std::endl;
  m->set_start(point);
  // print map on file
  std::ofstream map_file;
  map_file.open(map_path);
  // print wkt
  map_file << boost::geometry::wkt(m->_map) << std::endl;

  if (m->_show_graphics)
  {
    m->ShowMap();
  }

  rclcpp::sleep_for(std::chrono::seconds(1));

  // Monitor execution time
  auto time_start = rclcpp::Clock().now();
  double radius = 0.5;
  RRTStarDubinsPlan plan(m, radius);
  Path final_path = plan._run();

  // Check path validity
  cout << "IS PATH VALID?: " << (m->DubinsCollision(final_path) ? "NO" : "YES")
       << endl;

  // Output path for python visualisation
  print_path_on_file(m->pt_start, m->pt_end, final_path);

  auto time_end = rclcpp::Clock().now();
  auto time_diff = time_end - time_start;
  cout << "Planning time: " << time_diff.seconds() << " seconds" << endl;
  rclcpp::shutdown();
  cout << "Done!" << endl;
  return 0;
}