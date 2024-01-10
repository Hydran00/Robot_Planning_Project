#include "planner/rrt/rrt_star_dubins_plan.hpp"

#include "planner/dubins/dubins.h"

RRTStarDubinsPlan::RRTStarDubinsPlan(std::shared_ptr<MapInfo> &map_info,
                                     double radius)
    : MotionPlanning(map_info)
{
  _display = map_info->_show_graphics;
  _rrt.set_root(MotionPlanning::_pt_start);
  _radius = radius;
}

KDPoint RRTStarDubinsPlan::_GenerateRandPoint(void)
{
  unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
  std::default_random_engine generator(seed);
  std::uniform_int_distribution<int> dis_s(0, 9);
  // Epsilon greedy sampling
  if (dis_s(generator) < 2)
  {
    return MotionPlanning::_pt_end;
    // std::uniform_int_distribution<int> dis_s; TODO wtf ???????
  }
  else
  {
    // sample from the square embedding the map
    std::uniform_real_distribution<> dis_x((MotionPlanning::_map_info->min_x),
                                           (MotionPlanning::_map_info->max_x));
    std::uniform_real_distribution<> dis_y((MotionPlanning::_map_info->min_y),
                                           (MotionPlanning::_map_info->max_y));
    std::uniform_real_distribution<double> dis_yaw(0.0, 2 * M_PI);
    while (true)
    {
      // TODO: check if double sampling works
      double x = dis_x(generator);
      double y = dis_y(generator);
      double theta = dis_yaw(generator);
      KDPoint p = {double(x), double(y), double(theta)};
      if (!MotionPlanning::_map_info->Collision(p))
      {
        return p;
      }
    }
  }
}

std::vector<KDPoint> RRTStarDubinsPlan::_ReconstrucPath(void)
{
  std::vector<KDPoint> path;
  KDPoint p = MotionPlanning::_pt_end;
  while (p != MotionPlanning::_pt_start)
  {
    path.push_back(p);
    p = std::get<0>(_rrt.GetParent(p));
  }
  path.push_back(p);
  return path;
}

Path RRTStarDubinsPlan::_run(void)
{
  int n = 0;
  while (true)
  {
    KDPoint q_rand = _GenerateRandPoint();
    KDPoint q_near = _rrt.SearchNearestVertex(q_rand);
    // Returns tuple with (X, Y, Cost, Symbolic Path)
    auto dubins_best_path =
        get_dubins_best_path_and_cost(q_near, q_rand, _radius, 0.1);
    Linestring best_path;
    for (size_t i = 0; i < std::get<0>(dubins_best_path).size(); ++i)
    {
      best_path.push_back(point_xy(std::get<0>(dubins_best_path)[i],
                                   std::get<1>(dubins_best_path)[i]));
    }

    if (!boost::geometry::within(best_path, MotionPlanning::_map_info->_map))
    {
      continue;
    }
    // rclcpp::sleep_for(std::chrono::milliseconds(20));
    // TODO  probably we should redefine metric for dubins
    std::tuple<std::vector<double>, std::vector<double>> real_path =
        std::make_tuple(std::get<0>(dubins_best_path), std::get<1>(dubins_best_path));
    _rrt.Add(q_rand, q_near, std::get<3>(dubins_best_path), real_path);

    // TODO check radius->was 5.0
    // _rrt.DubinsRewire(
    //     q_near, 1.0,
    //     [&](std::tuple<std::vector<double>, std::vector<double>> &path) {
    //       return MotionPlanning::_map_info->DubinsCollision(path);
    //     },
    //     _radius);

    if (MotionPlanning::_display)
    {
      MotionPlanning::_map_info->set_rrt_dubins(_rrt, ++n);
    }

    // TODO->was 1
    if (Distance(q_rand, MotionPlanning::_pt_end) < 0.5)
    {
      Path total_path;
      KDPoint point = q_rand;
      while (point != MotionPlanning::_pt_start)
      {
        std::cout << "adding point :" << point[0] << ", " << point[1] << ", " <<point[2] << std::endl;
        auto tuple = _rrt.GetParent(point);
        Path p = std::get<3>(tuple);
        std::get<0>(total_path).insert(std::get<0>(total_path).begin(), std::get<0>(p).begin(), std::get<0>(p).end());
        std::get<1>(total_path).insert(std::get<1>(total_path).begin(), std::get<1>(p).begin(), std::get<1>(p).end());
        point = std::get<0>(tuple);
      }
      // Add last path
      std::get<0>(total_path).insert(std::get<0>(total_path).end(), std::get<0>(dubins_best_path).begin(), std::get<0>(dubins_best_path).end());
      std::get<1>(total_path).insert(std::get<1>(total_path).end(), std::get<1>(dubins_best_path).begin(), std::get<1>(dubins_best_path).end());

      // Print path
      // for (size_t i = 0; i < std::get<0>(total_path).size(); ++i)
      // {
      //   std::cout << std::get<0>(total_path)[i] << ", "
      //             << std::get<1>(total_path)[i] << std::endl;
      // }
      MotionPlanning::_map_info->set_dubins_path(total_path);
      return total_path;
    }
  }
}