#include "planner/rrt/utils/rrt.hpp"

#include <unistd.h>
#define VELOCITY 0.2
#define TIME_LIMIT 5000.0

void RRT::set_root(KDPoint &p) {
  _root.assign(p.begin(), p.end());
  _rrt.push_back(std::make_pair(_root, 0));
}

KDPoint RRT::SearchNearestVertex(KDPoint &q_rand, int iter) {
  std::vector<double> d;
  // extract random point with increasing probability
  // unsigned seed =
  // std::chrono::system_clock::now().time_since_epoch().count();
  std::uniform_int_distribution<int> epsilon_greedy_prob(0, 100);
  if (epsilon_greedy_prob(generator) < (float)0.005 * iter) {
    std::uniform_int_distribution<int> dis_s(0, _rrt.size() - 1);
    int idx = dis_s(generator);
    return _rrt[idx].first;
  }
  double distance;
  for (auto pair : _rrt) {
    distance = Distance(pair.first, q_rand);
    if (Cost(pair.first, false) + distance > VELOCITY * TIME_LIMIT) {
      d.push_back(std::numeric_limits<double>::infinity());
    } else {
      d.push_back((distance) + 0.95 * Cost(pair.first, true));
    }
  }
  int i = std::min_element(d.begin(), d.end()) - d.begin();
  return _rrt[i].first;
}

KDPoint RRT::CalcNewPoint(KDPoint &q_near, KDPoint &q_rand) {
  if (Distance(q_near, q_rand) < branch_lenght) {
    return q_rand;
  }

  double angle = std::atan2(q_rand[1] - q_near[1], q_rand[0] - q_near[0]);
  double x_new = q_near[0] + branch_lenght * std::cos(angle);
  double y_new = q_near[1] + branch_lenght * std::sin(angle);
  KDPoint p = {x_new, y_new};
  return p;
}

void RRT::Add(KDPoint &q_new, KDPoint &q_near) {
  int i = std::find_if(_rrt.begin(), _rrt.end(),
                       [&](std::pair<KDPoint, int> &pair) {
                         return (pair.first == q_near);
                       }) -
          _rrt.begin();
  _rrt.push_back(std::make_pair(q_new, i));
}

KDPoint RRT::GetParent(KDPoint &p) {
  auto it = std::find_if(
      _rrt.begin(), _rrt.end(),
      [&](std::pair<KDPoint, int> &pair) { return (pair.first == p); });
  return _rrt[it->second].first;
}

double RRT::Cost(KDPoint &point, bool consider_victims) {
  std::vector<std::tuple<KDPoint, double>> victims_list = victims;
  KDPoint p = point;
  double cost = 0.0;
  while (p != _root) {
    // checks if p is a victim
    // std::cout<<"| loop "<<p[0]<<" , "<<p[1]<<"|"<<std::endl;

    if (consider_victims) {
      auto it = std::find_if(victims_list.begin(), victims_list.end(),
                             [&](std::tuple<KDPoint, double> &victim) {
                               return (std::get<0>(victim) == p);
                             });
      if (it != victims_list.end()) {
        cost -= std::get<1>(*it);
        victims_list.erase(it);
      }
    }
    KDPoint f = GetParent(p);
    cost += Distance(p, f);
    p = f;
  }
  return cost;
}

void RRT::Rewire(KDPoint &p, double r,
                 std::function<bool(std::vector<KDPoint> &branch)> Collision) {
  std::vector<KDPoint> nears;
  auto it_p = std::find_if(
      _rrt.begin(), _rrt.end(),
      [&](std::pair<KDPoint, int> &pair) { return (pair.first == p); });
  std::for_each(_rrt.begin(), _rrt.end(), [&](std::pair<KDPoint, int> &pair) {
    std::vector<KDPoint> branch = {pair.first, p};
    if ((pair.first != p) && (Distance(pair.first, p) < r) &&
        (!Collision(branch))) {
      nears.push_back(pair.first);
    }
  });

  // checks if p is a victim
  auto it_vict = std::find_if(victims.begin(), victims.end(),
                              [&](std::tuple<KDPoint, double> &victim) {
                                return (std::get<0>(victim) == p);
                              });

  double last_segment_cost = 0.0;

  for (auto pt : nears) {
    // if (pt == GetParent(p)) {
    //   continue;
    // }
    // if (pt == _root || GetParent(pt) == _root) {
    //   continue;
    // }
    KDPoint p_copy = p;
    bool is_anchestor = false;
    while(p_copy != _root){
      if(p_copy == pt){
        is_anchestor = true;
        break;
      }
      p_copy = GetParent(p_copy);
    }
    if(is_anchestor){
      // std::cout << "Anchestor found! Skipping..."<<std::endl; 
      continue;
    }

    // std::cout << "Checking node " << pt[0] << " , " << pt[1] << std::endl;
    last_segment_cost = Distance(p, pt);
    // std::cout << "DEBUG 0"<< std::endl;

    // avoid rewiring if the new total path is too long to be travelled
    double distance = last_segment_cost;
    if (distance > VELOCITY * TIME_LIMIT) {
      continue;
    }
    // // considers the fact that p could be a victim
    // if (it_vict != victims.end()) {
    //   last_segment_cost -= std::get<1>(*it_vict);
    // }
    
    // if (Cost(pt, true) + last_segment_cost < Cost(p, true)) {

    // checks for the cost improvement
    // std::cout << "Checking node " << pt[0] << " , " << pt[1] << " with cost "
    //           << Cost(pt, false) << " and " << p[0] << " , " << p[1] << " with "
    //           << Cost(p, false) << std::endl;
    std::vector<KDPoint> branch = {pt, p};
    if (Cost(pt, true) > Cost(p, true) + last_segment_cost &&
        !Collision(branch)) {
      // std::cout << "Rewiring node "<< pt[0] << " , " << pt[1] << " to " <<
      // p[0]
      // << " , " << p[1] << std::endl;
      // int idx = std::find_if(_rrt.begin(), _rrt.end(),
      //                        [&](std::pair<KDPoint, int> &pair) {
      //                          return (pair.first == pt);
      //                        }) -
      //           _rrt.begin();
      // it_p->second = idx;

      // pt becomes son of p
      // std::cout << "Parent of " << pt[0] << " , " << pt[1] << " was "
      //           << GetParent(pt)[0] << " , " << GetParent(pt)[1] << std::endl;
      // std::get<1>(*std::find_if(_rrt.begin(), _rrt.end(),
      //                           [&](std::pair<KDPoint, int> &pair) {
      //                             return (pair.first == pt);
      //                           })) = it_p - _rrt.begin();

      auto it = std::find_if(_rrt.begin(), _rrt.end(),
                            [&](std::pair<KDPoint, int> &pair) {
                              return (pair.first == pt);
                            });
      std::get<1>(*it) = it_p - _rrt.begin();



      // std::cout << "Parent of " << pt[0] << " , " << pt[1] << " becomes "
      //           << GetParent(pt)[0] << " , " << GetParent(pt)[1]
      //           << "\n-------------------------" << std::endl;
    }
  }
}

bool RRT::PathOptimisation(
    KDPoint &current_node_, KDPoint &node_end,
    std::function<bool(std::vector<KDPoint> &path)> Collision) {
  // std::cout << "\033[1;35mStart optimising!\033[0m" << std::endl;
  // std::cout << "---------------------" << std::endl;

  bool is_path_improved = false;

  // Node we start looking from
  KDPoint current_node = current_node_;

  // Node we want to reduce the cost
  auto node_to_opt = node_end;

  // iterates from node_new to root
  while (current_node != _root) {
    // get parent of the current node
    KDPoint node_parent = GetParent(current_node);

    double new_cost =
        Cost(node_parent, true) + Distance(node_parent, node_to_opt);
    // checks if node_to_opt is a victim
    auto it = std::find_if(victims.begin(), victims.end(),
                           [&](std::tuple<KDPoint, double> &victim) {
                             return (std::get<0>(victim) == node_to_opt);
                           });
    if (it != victims.end()) {
      new_cost -= std::get<1>(*it);
    }

    std::cout << "Trying to skip node " << current_node[0] << " , "
              << current_node[1] << " and connect " << node_parent[0] << " "
              << node_parent[1] << " to " << node_to_opt[0] << " "
              << node_to_opt[1] << std::endl;
    std::vector<KDPoint> new_path = {node_parent, node_to_opt};
    if (new_cost < Cost(node_to_opt, true) && !Collision(new_path) - 0.1) {
      // if the Dubins is collision free, we can optimise the path
      std::cout << "\033[1;32mOptimisation found!\033[0m" << std::endl;

      int idx = std::find_if(_rrt.begin(), _rrt.end(),
                             [&](std::pair<KDPoint, int> &pair) {
                               return (std::get<0>(pair) == node_parent);
                             }) -
                _rrt.begin();
      auto it_node_to_opt = std::find_if(
          _rrt.begin(), _rrt.end(), [&](std::pair<KDPoint, int> &pair) {
            return (std::get<0>(pair) == node_to_opt);
          });

      std::get<1>(*it_node_to_opt) = idx;  // std::get<1>(*it_current_node);
      // std::cout << "Parent of " << node_to_optce)[1] << std::endl;
      is_path_improved = true;
    } else {
      std::cout << "\033[1;33mOptimisation not found!\033[0m" << std::endl;
      // std::cout << "Reason-> Cost improvement:"
      //           << ((new_cost < Cost(node_to_opt, dubins_radius, true))
      //                   ? "true"
      //                   : "false")
      //           << " | Path free: "
      //           << (!DubinsCollision(std::get<0>(dubins_opt_path1)) ? "true"
      //                                                               :
      //                                                               "false")
      //           << std::endl;
      // std::cout << "Costs are : " << new_cost << " and "
      //           << Cost(node_to_opt, dubins_radius, false) << std::endl;
      node_to_opt = current_node;
    }
    // std::cout << "---------------------" << std::endl;
    current_node = node_parent;
  }
  std::cout << "Path optimisation done!" << std::endl;
  return is_path_improved;
}