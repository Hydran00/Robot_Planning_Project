#include <unistd.h>

#include <fstream>
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

void print_path_on_file(Path path) {
  std::string file_path =
      ament_index_cpp::get_package_share_directory("planner") +
      "/data/dubins_path.txt";
  std::remove(file_path.c_str());
  std::ofstream fout;
  fout.open(file_path);
  for (size_t i = 0; i < std::get<0>(path).size(); i++) {
    fout << std::get<0>(path)[i] << ", " << std::get<1>(path)[i] << std::endl;
  }
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto m = std::make_shared<MapInfo>();

  sleep(0.5);
  RCLCPP_INFO(m->get_logger(), "Waiting for obstacles, borders and gates...");
  while (!m->obstacles_received_ || !m->borders_received_ ||
         !m->gates_received_) {
    rclcpp::spin_some(m);
  }

  KDPoint point = {0, 0, 0};
  while (m->Collision(point)) {
    point[0] += 0.1;
  }
  m->set_start(point);

  if (m->_show_graphics) {
    m->ShowMap();
  }
  rclcpp::sleep_for(std::chrono::seconds(1));
  std::vector<KDPoint> path;

  // Monitor execution time
  auto time_start = rclcpp::Clock().now();
  double radius = 0.5;
  RRTStarDubinsPlan plan(m, radius);
  Path PATH = plan._run();

  std::string file_path =
      ament_index_cpp::get_package_share_directory("planner") +
      "/data/dubins_path.txt";
  
  // cout << "IS PATH VALID?: "
  //      << (boost::geometry::within(l, m->_map) ? "YES" : "NO") << endl;

  // // print path
  // if (!path.empty()) {
  //   m->set_dubins_path(full_dubins_path);
  // }
  auto time_end = rclcpp::Clock().now();
  auto time_diff = time_end - time_start;
  cout << "Planning time: " << time_diff.seconds() << " seconds" << endl;
  rclcpp::shutdown();
  cout << "Done!" << endl;
  return 0;
}