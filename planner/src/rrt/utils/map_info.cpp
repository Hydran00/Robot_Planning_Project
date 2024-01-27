#include "planner/rrt/utils/map_info.hpp"

#include <boost/geometry/strategies/cartesian/buffer_point_circle.hpp>

MapInfo::MapInfo() : Node("map"), _pub_i(0) {
  this->_marker_pub = create_publisher<visualization_msgs::msg::Marker>(
      "visualization_marker", 10000);
  this->declare_parameter("show_graphics", true);
  this->_show_graphics = this->get_parameter("show_graphics").as_bool();
  RCLCPP_INFO(this->get_logger(), "show_graphics: %s",
              this->_show_graphics ? "true" : "false");

  obstacles_received_ = false;
  borders_received_ = false;
  gates_received_ = false;

  const auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_custom);
  subscription_obstacles_ =
      this->create_subscription<obstacles_msgs::msg::ObstacleArrayMsg>(
          "/obstacles", qos,
          std::bind(&MapInfo::obstacles_cb, this, std::placeholders::_1));
  subscription_borders_ =
      this->create_subscription<geometry_msgs::msg::PolygonStamped>(
          "/borders", qos,
          std::bind(&MapInfo::borders_cb, this, std::placeholders::_1));
  subscription_gates_ =
      this->create_subscription<geometry_msgs::msg::PoseArray>(
          "/gate_position", qos,
          std::bind(&MapInfo::gate_cb, this, std::placeholders::_1));
  // TODO: add subscription for start point

  RCLCPP_INFO(this->get_logger(), "Node started");
}

MapInfo::~MapInfo() {}

void MapInfo::obstacles_cb(const obstacles_msgs::msg::ObstacleArrayMsg &msg) {
  subscription_obstacles_.reset();
  this->set_obstacle(msg);
  this->obstacles_received_ = true;
  RCLCPP_INFO(this->get_logger(), "\033[1;32mObstacles received! \033[0m");
}
void MapInfo::borders_cb(const geometry_msgs::msg::PolygonStamped &msg) {
  subscription_borders_.reset();
  std::vector<KDPoint> points;
  for (auto p : msg.polygon.points) {
    KDPoint pt = {p.x, p.y};
    points.push_back(pt);
  }
  KDPoint pt = {msg.polygon.points[0].x, msg.polygon.points[0].y};
  points.push_back(pt);
  this->set_boundary(points);
  this->borders_received_ = true;
  RCLCPP_INFO(this->get_logger(), "\033[1;32mMap borders received! \033[0m");
}
void MapInfo::gate_cb(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
  // unsubscribe
  subscription_gates_.reset();
  KDPoint end = {msg->poses[0].position.x, msg->poses[0].position.y};
  set_end(end);
  this->gates_received_ = true;
  RCLCPP_INFO(this->get_logger(), "\033[1;32mGate received! \033[0m");
}

/////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////
void MapInfo::set_boundary(std::vector<KDPoint> &points) {
  _line_boundary.header.frame_id = "map";
  _line_boundary.header.stamp = now();
  _line_boundary.action = visualization_msgs::msg::Marker::ADD;
  _line_boundary.ns = "map";
  _line_boundary.id = _id_boundary;
  _line_boundary.type = visualization_msgs::msg::Marker::LINE_STRIP;
  _line_boundary.pose.orientation.w = 1.0;
  _line_boundary.scale.x = 0.1;
  // _line_boundary.scale.y = 0.1;
  _line_boundary.color.a = 1.0;

  _line_boundary.points.clear();
  // fill the msg with the map
  for (auto p : points) {
    // ROS2 Point
    geometry_msgs::msg::Point p_ros;
    p_ros.x = p[0];
    p_ros.y = p[1];
    p_ros.z = 0;
    _line_boundary.points.push_back(p_ros);

    _map.outer().push_back(point_xy(p[0], p[1]));
  }
  // close ring
  _map.outer().push_back(_map.outer().front());
  // print polygon in wkt
  // std::cout << "polygon: " << boost::geometry::wkt(_map) << std::endl;

  // set the min and max values of the map for sampling
  for (auto p : points) {
    if (p[0] < min_x) {
      min_x = p[0];
    }
    if (p[0] > max_x) {
      max_x = p[0];
    }
    if (p[1] < min_y) {
      min_y = p[1];
    }
    if (p[1] > max_y) {
      max_y = p[1];
    }
  }
}

void MapInfo::set_obstacle(const obstacles_msgs::msg::ObstacleArrayMsg &msg) {
  int i = 11;
  int obs_counter = 0;
  _map.inners().resize(msg.obstacles.size());
  RCLCPP_INFO(this->get_logger(), "Found %d obstacles",
              (int)msg.obstacles.size());

  for (auto obs : msg.obstacles) {
    visualization_msgs::msg::Marker marker;
    marker.points.clear();
    marker.header.frame_id = "map";
    marker.header.stamp = now();
    marker.ns = "map";
    marker.id = _id_obstacle + (i++);
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.color.r = 0.0;
    marker.color.a = 1.0;

    if (obs.radius != 0.0) {
      // Adding circles
      // marker.type = visualization_msgs::msg::Marker::CYLINDER;
      // marker.pose.position.x = obs.polygon.points[0].x;
      // marker.pose.position.y = obs.polygon.points[0].y;
      // marker.pose.position.z = obs.polygon.points[0].z;
      // marker.scale.x = obs.radius * 2;
      // marker.scale.y = obs.radius * 2;
      // marker.scale.z = 1;
      // // Declare the point_circle strategy
      // boost::geometry::strategy::buffer::point_circle point_strategy(360);
      // boost::geometry::strategy::buffer::distance_symmetric<double>
      //     distance_strategy(obs.radius);
      // boost::geometry::strategy::buffer::join_round join_strategy;
      // boost::geometry::strategy::buffer::end_round end_strategy;
      // boost::geometry::strategy::buffer::side_straight side_strategy;

      // boost::geometry::model::multi_point<point> mp = {
      //     point_xy(obs.polygon.points[0].x, obs.polygon.points[0].y)};

    }

    else {
      // Adding rectangles
      marker.type = visualization_msgs::msg::Marker::CUBE;
      marker.pose.position.x =
          (obs.polygon.points[0].x + obs.polygon.points[2].x) / 2;
      marker.pose.position.y =
          (obs.polygon.points[0].y + obs.polygon.points[2].y) / 2;
      marker.pose.position.z = 0;
      marker.scale.x = abs(obs.polygon.points[0].x - obs.polygon.points[2].x);
      marker.scale.y = abs(obs.polygon.points[0].y - obs.polygon.points[2].y);
      marker.scale.z = 1;

      _obstacle_array.markers.push_back(marker);
      for (int j = 0; j < (int)obs.polygon.points.size(); j++) {
        _map.inners()[obs_counter].push_back(
            point_xy(obs.polygon.points[j].x, obs.polygon.points[j].y));
      }
      // close ring
      _map.inners()[obs_counter].push_back(_map.inners()[obs_counter].front());
    }
    obs_counter++;
  }
// rclcpp::sleep_for(std::chrono::milliseconds(1000));
// std::cout << "Polygon with obstacles: \n"
//           << boost::geometry::wkt(_map) << std::endl;
}

void MapInfo::set_start(KDPoint &point) {
  pt_start.assign(point.begin(), point.end());

  _m_start.header.stamp = now();
  _m_start.header.frame_id = "map";
  _m_start.action = visualization_msgs::msg::Marker::ADD;
  _m_start.ns = "map";
  _m_start.id = _id_start;
  _m_start.type = visualization_msgs::msg::Marker::POINTS;
  _m_start.pose.orientation.w = 1.0;
  _m_start.scale.x = 0.4;
  _m_start.scale.y = 0.4;
  _m_start.color.g = 1.0;
  _m_start.color.a = 1.0;

  geometry_msgs::msg::Point p;
  p.x = point[0];
  p.y = point[1];
  p.z = 0;

  _m_start.points.clear();
  _m_start.points.push_back(p);
}

void MapInfo::set_end(KDPoint &point) {
  pt_end.assign(point.begin(), point.end());

  _m_end.header.stamp = now();
  _m_end.header.frame_id = "map";
  _m_end.action = visualization_msgs::msg::Marker::ADD;
  _m_end.ns = "map";
  _m_end.id = _id_end;
  _m_end.type = visualization_msgs::msg::Marker::POINTS;
  _m_end.pose.orientation.w = 1.0;
  _m_end.scale.x = 0.4;
  _m_end.scale.y = 0.4;
  _m_end.color.r = 1.0;
  _m_end.color.a = 1.0;

  geometry_msgs::msg::Point p;
  p.x = point[0];
  p.y = point[1];
  p.z = 0;
  _m_end.points.clear();
  _m_end.points.push_back(p);
}

void MapInfo::set_path(std::vector<KDPoint> &path) {
  _m_path.header.frame_id = "map";
  _m_path.header.stamp = now();
  _m_path.action = visualization_msgs::msg::Marker::ADD;
  _m_path.ns = "map";
  _m_path.id = _id_path;
  _m_path.type = visualization_msgs::msg::Marker::LINE_STRIP;
  _m_path.pose.orientation.w = 1.0;
  // show path map on top (avoid graphical glitches)
  _m_path.pose.position.z += 0.05;
  _m_path.scale.x = 0.1;
  _m_path.scale.y = 0.1;
  _m_path.color.r = 1.0;
  _m_path.color.a = 1.0;

  _m_path.points.clear();
  for (auto p : path) {
    geometry_msgs::msg::Point p_;
    p_.x = p[0];
    p_.y = p[1];
    p_.z = 0;
    _m_path.points.push_back(p_);
  }
  _marker_pub->publish(_m_path);
}

// void MapInfo::set_dubins_path(
//     std::tuple<std::vector<double>, std::vector<double>> path)
void MapInfo::set_dubins_path(std::vector<KDPoint> &path) {
  _m_path.header.frame_id = "map";
  _m_path.header.stamp = now();
  _m_path.action = visualization_msgs::msg::Marker::ADD;
  _m_path.ns = "map";
  _m_path.id = _id_path;
  _m_path.type = visualization_msgs::msg::Marker::LINE_STRIP;
  _m_path.pose.orientation.w = 1.0;
  // show path map on top (avoid graphical glitches)
  _m_path.pose.position.z = 0.05;
  _m_path.scale.x = 0.1;
  _m_path.scale.y = 0.1;
  _m_path.color.r = 1.0;
  _m_path.color.a = 1.0;

  _m_path.points.clear();
  geometry_msgs::msg::Point p_;

  for (size_t i = 0; i < path.size(); ++i) {
    p_.x = path[i][0];
    p_.y = path[i][1];
    p_.z = 0;
    _m_path.points.push_back(p_);
  }
  _marker_pub->publish(_m_path);
}

void MapInfo::set_openlist(std::vector<KDPoint> &points) {
  _m_openlist.header.frame_id = "map";
  _m_openlist.header.stamp = now();
  _m_openlist.action = visualization_msgs::msg::Marker::ADD;
  _m_openlist.ns = "map";
  _m_openlist.id = _id_openlist;
  _m_openlist.type = visualization_msgs::msg::Marker::POINTS;
  _m_openlist.pose.orientation.w = 1.0;
  _m_openlist.scale.x = 0.3;
  _m_openlist.scale.y = 0.3;
  _m_openlist.color.b = 0.5;
  _m_openlist.color.g = 0.5;
  _m_openlist.color.a = 1.0;

  _m_openlist.points.clear();
  for (auto p : points) {
    geometry_msgs::msg::Point p_;
    p_.x = p[0];
    p_.y = p[1];
    p_.z = 0;
    _m_openlist.points.push_back(p_);
  }
  _marker_pub->publish(_m_openlist);
  _pub_i = (_pub_i + 1) % 10;
  if (_pub_i == 0) {
    // rclcpp::sleep_for(std::chrono::milliseconds(10));
    return;
  }
}

void MapInfo::set_closelist(std::vector<KDPoint> &points) {
  _m_closelist.header.frame_id = "map";
  _m_closelist.header.stamp = now();
  _m_closelist.action = visualization_msgs::msg::Marker::ADD;
  _m_closelist.ns = "map";
  _m_closelist.id = _id_closelist;
  _m_closelist.type = visualization_msgs::msg::Marker::POINTS;
  _m_closelist.pose.orientation.w = 1.0;
  _m_closelist.scale.x = 0.3;
  _m_closelist.scale.y = 0.3;
  _m_closelist.color.b = 1.0;
  _m_closelist.color.a = 1.0;

  _m_closelist.points.clear();
  for (auto p : points) {
    geometry_msgs::msg::Point p_;
    p_.x = p[0];
    p_.y = p[1];
    p_.z = 0;
    _m_closelist.points.push_back(p_);
  }
  _marker_pub->publish(_m_closelist);
  _pub_i = (_pub_i + 1) % 10;
  if (_pub_i == 0) {
    // rclcpp::sleep_for(std::chrono::milliseconds(10));
    return;
  }
}

void MapInfo::set_rand_points(std::vector<KDPoint> &points) {
  _m_rand_points.header.frame_id = "map";
  _m_rand_points.header.stamp = now();
  _m_rand_points.action = visualization_msgs::msg::Marker::ADD;
  _m_rand_points.ns = "map";
  _m_rand_points.id = _id_rand_points;
  _m_rand_points.type = visualization_msgs::msg::Marker::POINTS;
  _m_rand_points.pose.orientation.w = 1.0;
  _m_rand_points.scale.x = 0.3;
  _m_rand_points.scale.y = 0.3;
  _m_rand_points.color.b = 0.8;
  _m_rand_points.color.r = 0.8;
  _m_rand_points.color.a = 1.0;

  _m_rand_points.points.clear();
  for (auto p : points) {
    geometry_msgs::msg::Point p_;
    p_.x = p[0];
    p_.y = p[1];
    p_.z = 0;
    _m_rand_points.points.push_back(p_);
  }
  _marker_pub->publish(_m_rand_points);
}

void MapInfo::set_roadmap(
    std::vector<std::pair<KDPoint, std::vector<KDPoint>>> &road_map) {
  _m_roadmap.header.frame_id = "map";
  _m_roadmap.header.stamp = now();
  _m_roadmap.action = visualization_msgs::msg::Marker::ADD;
  _m_roadmap.ns = "map";
  _m_roadmap.id = _id_roadmap;
  _m_roadmap.type = visualization_msgs::msg::Marker::LINE_LIST;
  _m_roadmap.pose.orientation.w = 1.0;
  _m_roadmap.scale.x = 0.1;
  _m_roadmap.scale.y = 0.1;
  _m_roadmap.color.b = 0.3;
  _m_roadmap.color.r = 0.1;
  _m_roadmap.color.a = 1.0;

  _m_roadmap.points.clear();
  for (auto rm : road_map) {
    geometry_msgs::msg::Point p1;
    p1.x = rm.first[0];
    p1.y = rm.first[1];
    p1.z = 0;
    for (auto p : rm.second) {
      geometry_msgs::msg::Point p2;
      p2.x = p[0];
      p2.y = p[1];
      p2.z = 0;
      _m_roadmap.points.push_back(p1);
      _m_roadmap.points.push_back(p2);
    }
  }
  _marker_pub->publish(_m_roadmap);
}

void MapInfo::set_rrt(RRT &rrt, int n, KDPoint &rand) {
  _m_rand_point.header.frame_id = "map";
  _m_rand_point.header.stamp = now();
  _m_rand_point.action = visualization_msgs::msg::Marker::ADD;
  _m_rand_point.ns = "map";
  _m_rand_point.id = _id_rand_point;
  _m_rand_point.type = visualization_msgs::msg::Marker::POINTS;
  _m_rand_point.pose.orientation.w = 1.0;
  _m_rand_point.scale.x = 0.5;
  _m_rand_point.scale.y = 0.5;
  _m_rand_point.color.b = 1.0;
  _m_rand_point.color.a = 1.0;

  _m_rand_point.points.clear();
  geometry_msgs::msg::Point p;
  p.x = rand[0];
  p.y = rand[1];
  p.z = 0;
  _m_rand_point.points.push_back(p);

  _m_rrt.header.frame_id = "map";
  _m_rrt.header.stamp = now();
  _m_rrt.action = visualization_msgs::msg::Marker::ADD;
  _m_rrt.ns = "map";
  _m_rrt.id = _id_rrt + n;
  _m_rrt.type = visualization_msgs::msg::Marker::LINE_LIST;
  _m_rrt.pose.orientation.w = 1.0;
  _m_rrt.scale.x = 0.1;
  _m_rrt.scale.y = 0.1;
  _m_rrt.color.b = 0.5;
  _m_rrt.color.g = 0.5;
  _m_rrt.color.a = 1.0;

  _m_rrt.points.clear();
  for (auto p : rrt) {
    geometry_msgs::msg::Point p1, p2;
    p1.x = p[0];
    p1.y = p[1];
    p1.z = 0;
    KDPoint pt = rrt.GetParent(p);
    p2.x = pt[0];
    p2.y = pt[1];
    p2.z = 0;
    _m_rrt.points.push_back(p1);
    _m_rrt.points.push_back(p2);
  }
  _marker_pub->publish(_m_rand_point);
  _marker_pub->publish(_m_rrt);
  _pub_i = (_pub_i + 1) % 10;
  if (_pub_i == 0) {
    // rclcpp::sleep_for(std::chrono::milliseconds(10));
    return;
  }
}

void MapInfo::set_rrt_dubins(RRTDubins &rrt_dubins) {
  visualization_msgs::msg::Marker branch;
  branch.header.frame_id = "map";
  branch.header.stamp = now();
  branch.action = visualization_msgs::msg::Marker::ADD;
  branch.ns = "map";
  branch.id = _id_rrt;
  branch.type = visualization_msgs::msg::Marker::LINE_LIST;
  branch.pose.orientation.w = 1.0;
  branch.scale.x = 0.1;
  branch.scale.y = 0.1;
  branch.color.b = 0.5;
  branch.color.g = 0.5;
  branch.color.a = 1.0;
  branch.points.clear();

  visualization_msgs::msg::Marker m_points;
  m_points.header.frame_id = "map";
  m_points.header.stamp = now();
  m_points.action = visualization_msgs::msg::Marker::ADD;
  m_points.ns = "map";
  m_points.id = _id_rrt + 1000;
  m_points.type = visualization_msgs::msg::Marker::POINTS;
  m_points.pose.orientation.w = 1.0;
  m_points.scale.x = 0.1;
  m_points.scale.y = 0.1;
  m_points.color.b = 1;
  m_points.color.a = 1.0;
  m_points.points.clear();

  visualization_msgs::msg::Marker m_parents;
  m_parents.header.frame_id = "map";
  m_parents.header.stamp = now();
  m_parents.action = visualization_msgs::msg::Marker::ADD;
  m_parents.ns = "map";
  // change id to avoid overwriting
  m_parents.id = _id_rrt + 1001;
  m_parents.type = visualization_msgs::msg::Marker::LINE_LIST;
  m_parents.pose.orientation.w = 1.0;
  m_parents.scale.x = 0.05;
  m_parents.scale.y = 0.05;
  m_parents.color.r = 1.0;
  m_parents.color.g = 0.33;
  m_parents.color.a = 1.0;
  m_parents.points.clear();
  geometry_msgs::msg::Point p1, p2;
  for (std::tuple<KDPoint, int, SymbolicPath, std::vector<KDPoint>> tuple :
       rrt_dubins._rrt) {
    if (std::get<0>(tuple) != pt_start) {
      // Push first point and last
      p1.x = std::get<3>(tuple)[0][0];
      p1.y = std::get<3>(tuple)[0][1];
      p1.z = 0;
      m_points.points.push_back(p1);
      p2.x = std::get<3>(tuple).back()[0];
      p2.y = std::get<3>(tuple).back()[1];
      p2.z = 0;
      m_points.points.push_back(p2);
      // Plot the leaf
      for (size_t i = 0; i < std::get<3>(tuple).size() - 1; i++) {
        p1.x = std::get<3>(tuple)[i][0];
        p1.y = std::get<3>(tuple)[i][1];
        p1.z = 0;
        branch.points.push_back(p1);
        p2.x = std::get<3>(tuple)[i + 1][0];
        p2.y = std::get<3>(tuple)[i + 1][1];
        p2.z = 0;
        branch.points.push_back(p2);
      }
    }
  }

  _marker_pub->publish(branch);
  _marker_pub->publish(m_points);
  _marker_pub->publish(m_parents);

  // _pub_i = (_pub_i + 1) % 10;
  // if (_pub_i == 0) {
  //   return;
  // }
}

bool MapInfo::Collision(KDPoint &point) {
  return !boost::geometry::within(point_xy(point[0], point[1]), _map);
}

bool MapInfo::Collision(std::vector<KDPoint> &path) {
  Linestring l;
  for (size_t i = 0; i < path.size(); ++i) {
    l.push_back(point_xy(path[i][0], path[i][1]));
  }
  return !boost::geometry::within(l, _map);
}

void MapInfo::ShowMap(void) {
  while (_marker_pub->get_subscription_count() < 1) {
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }

  // force publishing

  for (int i = 0; i < 10; i++) {
    for (auto obstacle : _obstacle_array.markers) {
      _marker_pub->publish(obstacle);
    }
    _marker_pub->publish(_line_boundary);
    _marker_pub->publish(_m_start);
    _marker_pub->publish(_m_end);
    rclcpp::sleep_for(std::chrono::milliseconds(200));
  }
}