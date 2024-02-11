#include <unistd.h>

#include <fstream>
#include <future>
#include <iostream>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "planner/dubins/dubins.h"
#include "planner/voronoi/planners/voronoi_plan.hpp"
#include "planner/map_info_node.hpp"

using namespace std;

void print_path_on_file(std::vector<KDPoint> path)
{
  std::string file_path =
      ament_index_cpp::get_package_share_directory("planner") +
      "/data/final_path.txt";
  std::cout << "Printing path on file: " << file_path << std::endl;
  std::ofstream fout;
  fout.open(file_path, std::ios::app);
  // fout << std::endl;
  for (size_t i = 0; i < path.size(); i++)
  {
    fout << path[i][0] << ", " << path[i][1] << std::endl;
  }
  // close
  fout.close();
}

int main(int argc, char **argv)
{
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
         !m->victims_received_)
  {
    rclcpp::spin_some(m->get_node_base_interface());
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }
  RCLCPP_INFO(m->get_logger(), "\033[1;32m Map information received!\033[0m");

  // print map on file
  std::ofstream map_file;
  map_file.open(map_path);
  // print wkt
  map_file << boost::geometry::wkt(m->_map) << std::endl;
  // close
  map_file.close();

  // if (m->_show_graphics)
  // {
  m->ShowMap();
  // }
  // wait for the map to be shown
  rclcpp::sleep_for(std::chrono::milliseconds(2000));

  VoronoiPlan plan(m);
  cout << "Planner started!" << endl;
  // Monitor execution time
  auto time_start = rclcpp::Clock().now();
  std::pair<std::vector<KDPoint>, double> path = plan.GetPlan();
  std::vector<KDPoint> dubinised_final_path =
      dubinise_path(path.first, m->dubins_radius, 0.1);
  auto time_end = rclcpp::Clock().now();
  auto time_diff = time_end - time_start;
  cout << "Planning time: " << time_diff.seconds() << " seconds" << endl;

  // Check path validity
  cout << "IS PATH VALID?: " << (m->Collision(dubinised_final_path) ? "NO" : "YES")
       << endl;

  //   // Output path for python visualisation
  print_path_on_file(dubinised_final_path);
  // m->set_final_path(path.first);
  rclcpp::sleep_for(std::chrono::milliseconds(2000));
  m->set_final_path(dubinised_final_path);

  int i = 0;
  while (i < 100)
  {
    m->publish_path(dubinised_final_path);
    rclcpp::sleep_for(std::chrono::milliseconds(10));
    i++;
  }

  rclcpp::shutdown();
  return 0;
}