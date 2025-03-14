#include <iostream>
// #include <unistd.h>
#include <fstream>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "planner/map_info_node.hpp"
#include "planner/rrt/planners/rrt_plan.hpp"
#include "planner/rrt/planners/rrt_star_plan.hpp"
#include "planner/rrt/utils/kdtree.hpp"
#include "planner/rrt/utils/rrt.hpp"
// #include "planner/dubins/dubins.h"
using namespace std;

void print_path_on_file(std::vector<KDPoint> path) {
  std::string file_path =
      ament_index_cpp::get_package_share_directory("planner") +
      "/data/final_path.txt";
  std::cout << "Printing path on file: " << file_path << std::endl;
  std::ofstream fout;
  fout.open(file_path, std::ios::app);
  // fout << std::endl;
  for (size_t i = 0; i < path.size(); i++) {
    fout << path[i][0] << ", " << path[i][1] << std::endl;
  }
  // close
  fout.close();
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  std::string file_path =
      ament_index_cpp::get_package_share_directory("planner") +
      "/data/final_path.txt";
  std::remove(file_path.c_str());

  std::string map_path =
      ament_index_cpp::get_package_share_directory("planner") + "/data/map.txt";
  std::remove(file_path.c_str());

  auto m = std::make_shared<MapInfo>();

  RCLCPP_INFO(m->get_logger(), "Waiting for obstacles, borders and gates...");
  while (!m->start_received_ || !m->obstacles_received_ ||
         !m->borders_received_ || !m->gates_received_ ||
         !m->victims_received_) {
    rclcpp::spin_some(m->get_node_base_interface());
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }
  RCLCPP_INFO(m->get_logger(), "\033[1;32m Map information received!\033[0m");

  KDPoint point = {0, 0, 0};
  while (m->Collision(point)) {
    point[0] += 0.1;
  }
  std::cout << "Start is at " << point[0] << ", " << point[1] << std::endl;
  m->set_start(point);
  // print map on file
  std::ofstream map_file;
  std::cout << "Printing map on file: " << map_path << std::endl;
  map_file.open(map_path);
  // print wkt
  map_file << boost::geometry::wkt(m->_map) << std::endl;
  // close
  map_file.close();
  // if (m->_show_graphics)
  // {
  m->ShowMap();
  // }

  // rclcpp::sleep_for(std::chrono::milliseconds(100));

  // Monitor execution time
  auto time_start = rclcpp::Clock().now();
  std::cout << "Running RRT*" << std::endl;
  RRTStarPlan plan(m);

  std::cout << "\033[1;32mSeed is " << plan.seed << "\033[0m" << std::endl;

  std::tuple<std::vector<KDPoint>, double> final_path_cost = plan.run();
  std::vector<KDPoint> final_path = std::get<0>(final_path_cost);
  std::vector<KDPoint> dubinised_final_path =
      dubinise_path(final_path, m->dubins_radius, 0.1);
  auto time_end = rclcpp::Clock().now();
  auto time_diff = time_end - time_start;
  std::cout << "Plan completed" << std::endl;
  m->set_final_path(final_path, "b", 11);
  m->set_final_path(dubinised_final_path);
  rclcpp::sleep_for(std::chrono::milliseconds(1000));
  m->set_final_path(dubinised_final_path);

  print_path_on_file(dubinised_final_path);
  //   // Output path for python visualisation

  int i = 0;
  while (i < 100) {
    m->publish_path(dubinised_final_path);
    rclcpp::sleep_for(std::chrono::milliseconds(10));
    i++;
  }
  cout << "Planning time: " << time_diff.seconds() << " seconds" << endl;
  rclcpp::shutdown();
  cout << "Done!" << endl;
  return 0;
}