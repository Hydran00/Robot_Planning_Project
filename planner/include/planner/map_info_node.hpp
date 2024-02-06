#ifndef MAP_INFO_NODE_HPP_
#define MAP_INFO_NODE_HPP_

#include <random>
#include <tuple>
#include <vector>
// Boost geometry primitives
#include <boost/geometry.hpp>
#include <boost/geometry/algorithms/append.hpp>
#include <boost/geometry/geometries/geometries.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
// well known text representation of geometry
#include <boost/geometry/io/wkt/wkt.hpp>
// within to check if a point is inside a polygon
#include <boost/geometry/algorithms/within.hpp>
// intersection
#include <boost/geometry/algorithms/intersects.hpp>

#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "planner/rrt/utils/kdtree.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "obstacles_msgs/msg/obstacle_array_msg.hpp"
#include "obstacles_msgs/msg/obstacle_msg.hpp"
#include "rclcpp/rclcpp.hpp"
#include "planner/rrt/utils/rrt.hpp"
#include "planner/rrt/utils/rrt_dubins.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

// Offsetting parameters and utils
#define OFFSET 0.25 + 0.10  // half-shelfino width + epsilon

typedef boost::geometry::model::d2::point_xy<double> point_xy;
typedef boost::geometry::model::linestring<point_xy> Linestring;
typedef boost::geometry::model::polygon<point_xy> polygon;

static const rmw_qos_profile_t rmw_qos_profile_custom = {
    RMW_QOS_POLICY_HISTORY_KEEP_LAST,
    10,
    RMW_QOS_POLICY_RELIABILITY_RELIABLE,
    RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
    RMW_QOS_DEADLINE_DEFAULT,
    RMW_QOS_LIFESPAN_DEFAULT,
    RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
    RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
    false};

class MapInfo : public rclcpp::Node {
 private:
  enum MarkerID {
    _id_boundary = 0,
    _id_obstacle = 1,
    _id_start = 2,
    _id_end = 3,
    _id_path = 4,
    _id_openlist = 5,
    _id_closelist = 6,
    _id_rand_points = 7,
    _id_roadmap = 8,
    _id_rand_point = 9,
    _id_rrt = 10,
    _id_victims = 11,
    _id_victims_cost_text = 12
  };
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr _marker_pub;
  // rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
  // _markerarray_pub;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_start_;
  rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr
      subscription_borders_;
  rclcpp::Subscription<obstacles_msgs::msg::ObstacleArrayMsg>::SharedPtr
      subscription_obstacles_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr
      subscription_gates_;
  rclcpp::Subscription<obstacles_msgs::msg::ObstacleArrayMsg>::SharedPtr
      subscription_victims_;

  // geometry_msgs::msg::PolygonStamped borders_;
  // obstacles_msgs::msg::ObstacleArrayMsg obstacles_;
  // Dubins path publisher
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr _final_path_pub;

  // define the map as a polygon with holes
  visualization_msgs::msg::Marker _line_boundary;
  visualization_msgs::msg::MarkerArray _obstacle_array;
  visualization_msgs::msg::Marker _m_start;
  visualization_msgs::msg::Marker _m_end;
  visualization_msgs::msg::Marker _m_openlist;
  visualization_msgs::msg::Marker _m_closelist;
  visualization_msgs::msg::Marker _m_path;
  visualization_msgs::msg::Marker _m_rand_points;
  visualization_msgs::msg::Marker _m_roadmap;
  visualization_msgs::msg::Marker _m_rand_point;
  visualization_msgs::msg::Marker _m_rrt;
  visualization_msgs::msg::Marker _m_victims;
  visualization_msgs::msg::MarkerArray _m_victim_cost_text;
  KDTree _okdtree;

  int _pub_i;

 public:
  // circle creation
  boost::geometry::strategy::buffer::join_round circle_join_strategy_;

  // Polygon offsetting
  boost::geometry::strategy::buffer::point_square offsetting_point_strategy_;
  boost::geometry::strategy::buffer::join_miter offsetting_join_strategy_;

  boost::geometry::strategy::buffer::end_round end_strategy_;
  boost::geometry::strategy::buffer::side_straight side_strategy_;

  // ros params
  double _timeout;
  double _num_threads;
  std::string _planner_type;

  polygon _map;
  bool start_received_;
  bool obstacles_received_;
  bool borders_received_;
  bool gates_received_;
  bool victims_received_;
  double offset;
  double dubins_radius;

  double min_x, max_x, min_y, max_y;
  KDPoint pt_start;
  KDPoint pt_end;
  typedef std::tuple<KDPoint, double> Victim;
  std::vector<Victim> _victims;
  bool _show_graphics;
  MapInfo();
  ~MapInfo();

  void start_cb(const nav_msgs::msg::Odometry &msg);
  void obstacles_cb(const obstacles_msgs::msg::ObstacleArrayMsg &msg);
  void borders_cb(const geometry_msgs::msg::PolygonStamped &msg);
  void gate_cb(const geometry_msgs::msg::PoseArray::SharedPtr msg);
  void victims_cb(const obstacles_msgs::msg::ObstacleArrayMsg &msg);
  void set_boundary(std::vector<KDPoint> &points);
  void set_obstacle(const obstacles_msgs::msg::ObstacleArrayMsg &msg);
  void set_victims();
  void set_start(KDPoint &point);
  void set_end(KDPoint &point);
  void set_final_path(std::vector<KDPoint> &path);
  //   void set_dubins_path(std::vector<KDPoint> &path);
  //   void set_openlist(std::vector<KDPoint> &points);
  //   void set_closelist(std::vector<KDPoint> &points);
  //   void set_rand_points(std::vector<KDPoint> &points);
  //   void set_roadmap(
  //       std::vector<std::pair<KDPoint, std::vector<KDPoint>>> &road_map);
  void set_rrt(RRT &rrt, int n, KDPoint &rand);
  void set_rrt_dubins(RRTDubins &rrt_dubins);
  bool Collision(KDPoint &point);
  //   bool Collision(KDPoint &p1, KDPoint &p2);
  bool Collision(std::vector<KDPoint> &path);
  //   bool DubinsCollision(
  //   std::vector<KDPoint> &path);
  void ShowMap(void);

  // Publish final path
  void publish_path(std::vector<KDPoint> final_path);
};

#endif  // MAP_INFO_HPP_
