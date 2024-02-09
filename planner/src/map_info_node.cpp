#include "planner/map_info_node.hpp"

#include <unistd.h>

#include <boost/geometry/strategies/cartesian/buffer_point_circle.hpp>

MapInfo::MapInfo() : Node("map"), _pub_i(0) {
  this->_marker_pub = create_publisher<visualization_msgs::msg::Marker>(
      "visualization_marker", 10000);

  this->declare_parameter("planner_type", "not_specified");
  this->_planner_type = this->get_parameter("planner_type").as_string();

  this->declare_parameter("show_graphics", true);
  this->_show_graphics = this->get_parameter("show_graphics").as_bool();

  this->declare_parameter("dubins_radius", 0.5);
  this->dubins_radius = this->get_parameter("dubins_radius").as_double();

  this->declare_parameter("num_threads", 4);
  this->_num_threads = this->get_parameter("num_threads").as_int();

  this->declare_parameter("timeout", 10.0);
  this->_timeout = this->get_parameter("timeout").as_double();

  RCLCPP_INFO(this->get_logger(), "show_graphics: %s",
              this->_show_graphics ? "true" : "false");

  start_received_ = false;
  obstacles_received_ = false;
  borders_received_ = false;
  gates_received_ = false;
  victims_received_ = false;

  if(_planner_type != "rrt_star_dubins"){
    obstacle_offset = OFFSET + dubins_radius;
    map_offset = 0;//OFFSET; 

  }else{
    obstacle_offset = OFFSET;
    map_offset = 0;//OFFSET;
  }

  const auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_custom);
  subscription_start_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "shelfino0/odom", 10,
      std::bind(&MapInfo::start_cb, this, std::placeholders::_1));
  subscription_obstacles_ =
      this->create_subscription<obstacles_msgs::msg::ObstacleArrayMsg>(
          "obstacles", qos,
          std::bind(&MapInfo::obstacles_cb, this, std::placeholders::_1));
  subscription_borders_ =
      this->create_subscription<geometry_msgs::msg::PolygonStamped>(
          "borders", qos,
          std::bind(&MapInfo::borders_cb, this, std::placeholders::_1));
  subscription_gates_ =
      this->create_subscription<geometry_msgs::msg::PoseArray>(
          "gate_position", qos,
          std::bind(&MapInfo::gate_cb, this, std::placeholders::_1));
  subscription_victims_ =
      this->create_subscription<obstacles_msgs::msg::ObstacleArrayMsg>(
          "victims", qos,
          std::bind(&MapInfo::victims_cb, this, std::placeholders::_1));
  // TODO: add subscription for start point
  _final_path_pub =
      this->create_publisher<nav_msgs::msg::Path>("shelfino0/plan1", 10);
  RCLCPP_INFO(this->get_logger(), "Node started");
}

MapInfo::~MapInfo() {}

void MapInfo::start_cb(const nav_msgs::msg::Odometry &msg) {
  subscription_start_.reset();
  tf2::Quaternion q(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                    msg.pose.pose.orientation.z, msg.pose.pose.orientation.w);
  // extract yaw
  double roll, pitch, yaw;
  tf2::Matrix3x3 m(q);
  m.getRPY(roll, pitch, yaw);

  KDPoint start = {msg.pose.pose.position.x, msg.pose.pose.position.y, yaw};
  set_start(start);
  RCLCPP_INFO(this->get_logger(),
              "\033[1;32mInitial position received! \033[0m");
  start_received_ = true;
}

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
  tf2::Quaternion q(msg->poses[0].orientation.x, msg->poses[0].orientation.y,
                    msg->poses[0].orientation.z, msg->poses[0].orientation.w);
  // extract yaw
  double roll, pitch, yaw;
  tf2::Matrix3x3 m(q);
  m.getRPY(roll, pitch, yaw);

  KDPoint end = {msg->poses[0].position.x, msg->poses[0].position.y, yaw};
  set_end(end);
  this->gates_received_ = true;
  RCLCPP_INFO(this->get_logger(), "\033[1;32mGate received! \033[0m");
}
void MapInfo::victims_cb(const obstacles_msgs::msg::ObstacleArrayMsg &msg) {
  // unsubscribes
  subscription_victims_.reset();
  if (msg.obstacles.size() == 0) {
    this->victims_received_ = true;
    return;
  }
  for (size_t i = 0; i < msg.obstacles.size(); ++i) {
    KDPoint victim = {msg.obstacles[i].polygon.points[0].x,
                      msg.obstacles[i].polygon.points[0].y};
    double cost = (double)msg.obstacles[i].radius / 10;
    Victim v = std::make_tuple(victim, cost);
    _victims.push_back(v);
  }
  set_victims();
  this->victims_received_ = true;
  RCLCPP_INFO(this->get_logger(), "\033[1;32mVictims received! \033[0m");
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
  _line_boundary.scale.x = 0.5;
  // _line_boundary.scale.y = 0.1;
  _line_boundary.color.a = 1.0;
  _line_boundary.points.clear();
  // fill the msg with the map

  // Offsetting strategies and parameters
  //TODO check if the map_offset is correct
  boost::geometry::strategy::buffer::distance_symmetric<double>
      offsetting_distance_strategy(-map_offset);
  boost::geometry::model::multi_polygon<polygon> result;
  boost::geometry::model::multi_polygon<polygon> mp;
  mp.resize(1);
  for (auto p : points) {
    mp[0].outer().push_back(point_xy(p[0], p[1]));
  }
  // close ring
  mp[0].outer().push_back(mp[0].outer().front());
  // Offset the polygon and store it in result
  boost::geometry::buffer(mp, result, offsetting_distance_strategy,
                          side_strategy_, offsetting_join_strategy_,
                          end_strategy_, offsetting_point_strategy_);

  geometry_msgs::msg::Point p_ros;
  for (auto p : result[0].outer()) {
    _map.outer().push_back(point_xy(p.x(), p.y()));
    p_ros.x = p.x();
    p_ros.y = p.y();
    p_ros.z = 0;
    _line_boundary.points.push_back(p_ros);
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
  int i = 100;
  int obs_counter = 0;
  _map.inners().resize(msg.obstacles.size());
  RCLCPP_INFO(this->get_logger(), "Found %d obstacles",
              (int)msg.obstacles.size());

  // offsetting strategies
  // Circle creation
  boost::geometry::strategy::buffer::point_circle circle_point_strategy(30);

  // Polygon offsetting
  boost::geometry::strategy::buffer::distance_symmetric<double>
      offsetting_distance_strategy(obstacle_offset);
  boost::geometry::model::multi_polygon<polygon> result;

  for (auto obs : msg.obstacles) {
    visualization_msgs::msg::Marker marker;
    marker.points.clear();
    marker.header.frame_id = "map";
    marker.header.stamp = now();
    marker.ns = "map";
    marker.id = _id_obstacle + (i++);
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.color.g = 1.0;
    marker.color.a = 1.0;

    // Adds circles and rectangles
    if (obs.radius != 0.0) {
      // Adding circles
      marker.type = visualization_msgs::msg::Marker::CYLINDER;
      marker.pose.position.x = obs.polygon.points[0].x;
      marker.pose.position.y = obs.polygon.points[0].y;
      marker.pose.position.z = obs.polygon.points[0].z;
      marker.scale.x = obs.radius * 2 + obstacle_offset;
      marker.scale.y = obs.radius * 2 + obstacle_offset;
      marker.scale.z = 1.1;
      _obstacle_array.markers.push_back(marker);

      // Defines the circle center
      boost::geometry::model::multi_point<point_xy> mp = {
          point_xy(obs.polygon.points[0].x, obs.polygon.points[0].y)};
      boost::geometry::strategy::buffer::distance_symmetric<double>
          distance_strategy(obs.radius + obstacle_offset);
      // Creates the circle and stores it in result

      boost::geometry::buffer(mp, result, distance_strategy, side_strategy_,
                              circle_join_strategy_, end_strategy_,
                              circle_point_strategy);

      // Iterates over vertexes of the circle
      for (auto p : result[0].outer()) {
        _map.inners()[obs_counter].push_back(point_xy(p.x(), p.y()));
      }
      // close ring
      _map.inners()[obs_counter].push_back(_map.inners()[obs_counter].front());

    } else {
      // Adding rectangles
      marker.type = visualization_msgs::msg::Marker::CUBE;
      marker.pose.position.x =
          (obs.polygon.points[0].x + obs.polygon.points[2].x) / 2;
      marker.pose.position.y =
          (obs.polygon.points[0].y + obs.polygon.points[2].y) / 2;
      marker.pose.position.z = 0;

      marker.scale.x =
          abs(obs.polygon.points[0].x - obs.polygon.points[2].x) +
          obstacle_offset / abs(obs.polygon.points[0].x - obs.polygon.points[2].x);
      marker.scale.y =
          abs(obs.polygon.points[0].y - obs.polygon.points[2].y) +
          obstacle_offset / abs(obs.polygon.points[0].y - obs.polygon.points[2].y);
      marker.scale.z = 1.1;
      _obstacle_array.markers.push_back(marker);

      // OFFSETTING
      boost::geometry::model::multi_polygon<polygon> mp;
      mp.resize(1);
      for (auto p : obs.polygon.points) {
        mp[0].outer().push_back(point_xy(p.x, p.y));
      }
      // close ring
      mp[0].outer().push_back(mp[0].outer().front());
      // Offset the polygon and store it in result
      boost::geometry::buffer(mp, result, offsetting_distance_strategy,
                              side_strategy_, offsetting_join_strategy_,
                              end_strategy_, offsetting_point_strategy_);

      for (auto p : result[0].outer()) {
        _map.inners()[obs_counter].push_back(point_xy(p.x(), p.y()));
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

void MapInfo::set_victims() {
  _m_victims.header.stamp = now();
  _m_victims.header.frame_id = "map";
  _m_victims.action = visualization_msgs::msg::Marker::ADD;
  _m_victims.ns = "map";
  _m_victims.id = _id_victims;
  _m_victims.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  _m_victims.pose.orientation.w = 1.0;
  _m_victims.scale.x = 0.3;
  _m_victims.scale.y = 0.3;
  _m_victims.color.b = 1.0;
  _m_victims.color.a = 1.0;
  _m_victims.points.clear();

  int i = 150;
  geometry_msgs::msg::Point p;
  visualization_msgs::msg::Marker cost_text;
  cost_text.header.frame_id = "map";
  cost_text.header.stamp = now();
  cost_text.action = visualization_msgs::msg::Marker::ADD;
  cost_text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  cost_text.ns = "map";
  cost_text.scale.z = 0.5;
  cost_text.scale.y = 0.5;
  cost_text.scale.x = 0.5;
  cost_text.pose.orientation.w = 1.0;
  cost_text.color.r = 0.0;
  cost_text.color.g = 0.0;
  cost_text.color.b = 1.0;
  cost_text.color.a = 1.0;
  for (auto v : _victims) {
    p.x = std::get<0>(v)[0];
    p.y = std::get<0>(v)[1];
    p.z = 0;
    _m_victims.points.push_back(p);
    cost_text.pose.position.x = std::get<0>(v)[0];
    cost_text.pose.position.y = std::get<0>(v)[1] + 0.4;
    cost_text.pose.position.z = 0.6;
    char buffer[20];
    snprintf(buffer, sizeof(buffer), "%.2f",
             -roundf(std::get<1>(v) * 100) / 100);
    cost_text.text = buffer;
    cost_text.id = _id_victims_cost_text + i;
    _m_victim_cost_text.markers.push_back(cost_text);
    i++;
  }
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
  _m_end.pose.position.z = -1.0;
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

void MapInfo::set_final_path(std::vector<KDPoint> &path) {
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
  _m_path.scale.z = 0.01;
  _m_path.color.r = 1.0;
  _m_path.color.a = 1.0;

  _m_path.points.clear();
  float inc = 0.001;
  for (auto p : path) {
    geometry_msgs::msg::Point p_;
    p_.x = p[0];
    p_.y = p[1];
    p_.z = 0.2 + inc;
    // avoid glitches
    inc += 0.001;
    std::cout << "Plotting path: " << p[0] << " " << p[1] << std::endl;
    _m_path.points.push_back(p_);
  }
  _marker_pub->publish(_m_path);
}

void MapInfo::set_rrt(RRT &rrt, int n, KDPoint &rand) {
  _m_rand_point.header.frame_id = "map";
  _m_rand_point.header.stamp = now();
  _m_rand_point.action = visualization_msgs::msg::Marker::ADD;
  _m_rand_point.ns = "map";
  _m_rand_point.id = _id_rand_point;
  _m_rand_point.type = visualization_msgs::msg::Marker::SPHERE;
  _m_rand_point.scale.x = 0.5;
  _m_rand_point.scale.y = 0.5;
  _m_rand_point.scale.z = 0.5;
  _m_rand_point.color.r = 1.0;
  _m_rand_point.color.g = 0.4;
  _m_rand_point.color.a = 1.0;
  _m_rand_point.pose.position.x = rand[0];
  _m_rand_point.pose.position.y = rand[1];
  _m_rand_point.pose.position.z = 0;

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
  _marker_pub->publish(_m_rrt);
  _marker_pub->publish(_m_rand_point);
  _pub_i = (_pub_i + 1) % 10;
  if (_pub_i == 0) {
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
  // branch.color.r = 1.0;
  branch.color.b = 0.5;
  branch.color.g = 0.5;
  branch.color.a = 1.0;
  branch.points.clear();

  visualization_msgs::msg::Marker m_points;
  m_points.header.frame_id = "map";
  m_points.header.stamp = now();
  m_points.action = visualization_msgs::msg::Marker::ADD;
  m_points.ns = "map";
  m_points.id = _id_rrt + 130;
  m_points.type = visualization_msgs::msg::Marker::POINTS;
  m_points.pose.position.z = 0.1;
  m_points.scale.x = 0.1;
  m_points.scale.y = 0.1;
  // m_points.color.r = 1.0;
  // m_points.color.g = 0.5;
  m_points.color.b = 1.0;
  m_points.color.a = 1.0;
  m_points.points.clear();

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

void MapInfo::publish_path(std::vector<KDPoint> final_path) {
  // create path message
  nav_msgs::msg::Path nav_final_path;
  nav_final_path.header.frame_id = "map";
  nav_final_path.header.stamp = now();
  // convert final_path (KDPoint vector) to nav_final_path (PoseStamped array
  // message)
  auto compute_yaw = [](KDPoint &p1, KDPoint &p2) {
    return atan2(p2[1] - p1[1], p2[0] - p1[0]);
  };
  double yaw = 0.0;
  tf2::Quaternion quat;

  geometry_msgs::msg::PoseStamped point_pose_msg;
  for (size_t i = 0; i < final_path.size(); ++i) {
    // create point message
    point_pose_msg.header.frame_id = "map";
    // point_pose_msg.header.stamp = now();
    // populate point message with position
    point_pose_msg.pose.position.x = final_path[i][0];
    point_pose_msg.pose.position.y = final_path[i][1];
    point_pose_msg.pose.position.z = 0;
    if (i == final_path.size() - 1) {
      yaw = compute_yaw(final_path[i - 1], final_path[i]);
    } else {
      yaw = compute_yaw(final_path[i], final_path[i + 1]);
    }
    quat.setRPY(0, 0, yaw);
    // populate point message with orientation
    point_pose_msg.pose.orientation.x = quat.getX();
    point_pose_msg.pose.orientation.y = quat.getY();
    point_pose_msg.pose.orientation.z = quat.getZ();
    point_pose_msg.pose.orientation.w = quat.getW();
    // insert point message into path message
    nav_final_path.poses.push_back(point_pose_msg);
  }
  _final_path_pub->publish(nav_final_path);
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
    for (visualization_msgs::msg::Marker cost_text :
         _m_victim_cost_text.markers) {
      cost_text.header.stamp = now();
      cost_text.header.frame_id = "map";
      _marker_pub->publish(cost_text);
    }
    _marker_pub->publish(_line_boundary);
    _marker_pub->publish(_m_start);
    _marker_pub->publish(_m_end);
    _marker_pub->publish(_m_victims);
    rclcpp::sleep_for(std::chrono::milliseconds(50));
  }
}
