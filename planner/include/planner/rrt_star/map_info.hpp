#ifndef MAP_INFO_HPP_
#define MAP_INFO_HPP_

#include <vector>
#include <tuple>

// #include <boost/geometry.hpp>

#include "kdtree.hpp"
#include "rrt.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "obstacles_msgs/msg/obstacle_msg.hpp"
#include "obstacles_msgs/msg/obstacle_array_msg.hpp"

// Boost geometry primitives
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/algorithms/append.hpp> 
// well known text representation of geometry
#include <boost/geometry/io/wkt/wkt.hpp>
// within to check if a point is inside a polygon
#include <boost/geometry/algorithms/within.hpp>



typedef boost::geometry::model::d2::point_xy<double> point_xy;
typedef boost::geometry::model::polygon<point_xy> polygon;
static const rmw_qos_profile_t rmw_qos_profile_custom =
    {
        RMW_QOS_POLICY_HISTORY_KEEP_LAST,
        10,
        RMW_QOS_POLICY_RELIABILITY_RELIABLE,
        RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
        RMW_QOS_DEADLINE_DEFAULT,
        RMW_QOS_LIFESPAN_DEFAULT,
        RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
        RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
        false};

class MapInfo : public rclcpp::Node
{
private:
    enum MarkerID
    {
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
    };
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr _marker_pub;
    rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr subscription_borders_;
    rclcpp::Subscription<obstacles_msgs::msg::ObstacleArrayMsg>::SharedPtr subscription_obstacles_;
    bool obstacles_received_;
    bool borders_received_;
    // geometry_msgs::msg::PolygonStamped borders_;
    // obstacles_msgs::msg::ObstacleArrayMsg obstacles_;

    // define the map as a polygon with holes
    polygon _map;
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
    KDTree _okdtree;

    int _pub_i;

public:
    double min_x, max_x, min_y, max_y;
    KDPoint pt_start;
    KDPoint pt_end;
    bool _show_graphics;
    MapInfo();
    ~MapInfo();

    void obstacles_cb(const obstacles_msgs::msg::ObstacleArrayMsg &msg);
    void borders_cb(const geometry_msgs::msg::PolygonStamped &msg);
    void set_boundary(std::vector<KDPoint> &points);
    void set_obstacle(const obstacles_msgs::msg::ObstacleArrayMsg &msg);

    void set_start(KDPoint &point);
    void set_end(KDPoint &point);
    void set_path(std::vector<KDPoint> &path);
    void set_openlist(std::vector<KDPoint> &points);
    void set_closelist(std::vector<KDPoint> &points);
    void set_rand_points(std::vector<KDPoint> &points);
    void set_roadmap(std::vector<std::pair<KDPoint, std::vector<KDPoint>>> &road_map);
    void set_rrt(RRT &rrt, int n, KDPoint &rand);
    bool Collision(KDPoint &point);
    bool Collision(KDPoint &p1, KDPoint &p2);
    void ShowMap(void);
};

#endif // MAP_INFO_HPP_
