#include "planner/rrt/rrt_star_dubins_plan.hpp"

#include <fstream>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "planner/dubins/dubins.h"

RRTStarDubinsPlan::RRTStarDubinsPlan(std::shared_ptr<MapInfo> &map_info,
                                     double radius)
    : MotionPlanning(map_info), _rrt(map_info->_victims) {
  _display = map_info->_show_graphics;
  _rrt.set_root(MotionPlanning::_pt_start);
  _radius = radius;
}

KDPoint RRTStarDubinsPlan::_GenerateRandPoint(int iter) {
  unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
  std::default_random_engine generator(seed);
  int num_victims = MotionPlanning::_map_info->_victims.size();
  std::uniform_int_distribution<int> dis_s(0, 100);
  // Epsilon greedy sampling
  int extracted = dis_s(generator);
  if (extracted < 50 - iter * 0.02) {
    int idx = std::uniform_int_distribution<int>(0, num_victims - 1)(generator);
    KDPoint p = std::get<0>(MotionPlanning::_map_info->_victims[idx]);
    std::uniform_real_distribution<double> dis_yaw(0.0, 2 * M_PI - EPSILON);
    p.push_back(dis_yaw(generator));
    return p;
  } else {
    if (extracted < 99.9 - iter * 0.02) {
      // sample from the square embedding the map
      std::uniform_real_distribution<> dis_x(
          (MotionPlanning::_map_info->min_x),
          (MotionPlanning::_map_info->max_x));
      std::uniform_real_distribution<> dis_y(
          (MotionPlanning::_map_info->min_y),
          (MotionPlanning::_map_info->max_y));
      std::uniform_real_distribution<double> dis_yaw(0.0, 2 * M_PI - EPSILON);
      while (true) {
        double x = dis_x(generator);
        double y = dis_y(generator);
        double theta = dis_yaw(generator);
        KDPoint p = {double(x), double(y), double(theta)};
        if (!MotionPlanning::_map_info->Collision(p)) {
          return p;
        }
      }
    } else {
      std::uniform_real_distribution<double> dis_yaw(0.0, 2 * M_PI - EPSILON);
      KDPoint p = {MotionPlanning::_pt_end[0], MotionPlanning::_pt_end[1],
                   dis_yaw(generator)};
      return MotionPlanning::_pt_end;
    }
  }
}

std::vector<KDPoint> RRTStarDubinsPlan::_ReconstrucPath() {
  KDPoint p = MotionPlanning::_pt_end;
  std::vector<KDPoint> path;
  auto last_path = _rrt.GetPointPath(p);
  path.insert(path.begin(), last_path.begin(), last_path.end());
  while (MotionPlanning::_pt_start != p) {
    auto parent = _rrt.GetParent(p);
    std::vector<KDPoint> parent_path = std::get<3>(parent);
    path.insert(path.begin(), parent_path.begin(), parent_path.end());
    p = std::get<0>(parent);
  }
  return path;
}

void print_path_on_file1(std::vector<KDPoint> path) {
  std::string file_path =
      ament_index_cpp::get_package_share_directory("planner") +
      "/data/final_path0.txt";
  std::remove(file_path.c_str());
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

std::tuple<std::vector<KDPoint>, double>  RRTStarDubinsPlan::run(void) {
  int nodes_counter = 0;
  KDPoint p;
  int iter = 0;
  while (true) {
    iter++;
    p.clear();
    KDPoint q_rand = _GenerateRandPoint(iter);
    std::tuple<KDPoint, int, SymbolicPath, std::vector<KDPoint>> near_node =
        _rrt.SearchNearestVertex(q_rand, _radius, iter);

    KDPoint q_near = std::get<0>(near_node);

    // check that the new point is not already in the tree
    KDPoint p = q_near;
    bool already_visited = false;
    while (p != _rrt._root) {
      if (p[0] == q_rand[0] && p[1] == q_rand[1]) {
        already_visited = true;
        break;
      }
      p = std::get<0>(_rrt.GetParent(p));
    }
    if (already_visited) {
      // std::cout << "Already visited" << std::endl;
      // std::cout << "---------------------" << std::endl;
      continue;
    }

    std::tuple<std::vector<KDPoint>, double, std::vector<std::vector<double>>>
        dubins_best_path =
            get_dubins_best_path_and_cost(q_near, q_rand, _radius, 0.1);

    std::vector<KDPoint> new_path = std::get<0>(dubins_best_path);

    // Checks collisions
    if (MotionPlanning::_map_info->Collision(new_path)) {
      continue;
    }
    // Gets the new node
    std::tuple<KDPoint, int, SymbolicPath, std::vector<KDPoint>> new_node =
        _rrt.Add(q_rand, near_node, std::get<2>(dubins_best_path), new_path);

    // The rewire function is the difference between RRT and RRT*
    _rrt.Rewire(
        new_node, 100,
        [&](std::vector<KDPoint> &path) {
          return MotionPlanning::_map_info->Collision(path);
        },
        _radius);

    // Display the tree in Rviz2
    if (MotionPlanning::_display) {
      MotionPlanning::_map_info->set_rrt_dubins(_rrt);
    }

    // check if we are close to the end
    if (sqrt(pow(q_rand[0] - MotionPlanning::_pt_end[0], 2) +
             pow(q_rand[1] - MotionPlanning::_pt_end[1], 2)) < 0.0000001) {
      nodes_counter += 1;

      double cost1 = _rrt.Cost(new_node, _radius, false);
      // }
      std::vector<KDPoint> before_path = _ReconstrucPath();
      print_path_on_file1(before_path);
      // compute the final cost
      // keep optimising the path until it does not change anymore
      while (true) {
        auto parent = _rrt.GetParent(std::get<0>(new_node));
        if (_rrt.PathOptimisation(
                parent, new_node,
                [&](std::vector<KDPoint> &path) {
                  return MotionPlanning::_map_info->Collision(path);
                },
                _radius) == false) {
          break;
        }
      };
      std::cout << "\n\nFinal cost of node " << std::get<0>(new_node)[0] << ", "
                << std::get<0>(new_node)[1] << " is "
                << cost1 << std::endl;
      std::cout << "Final cost after optimisation of node "<< std::get<0>(new_node)[0] << ", "
                << std::get<0>(new_node)[1] << " is " 
                << _rrt.Cost(new_node, _radius, false) << std::endl;
      std::tuple<std::vector<KDPoint>, double> final_path_cost =
          std::make_tuple(_ReconstrucPath(), _rrt.Cost(new_node, _radius, true));
      return final_path_cost;
    }
    nodes_counter += 1;
  }
}
