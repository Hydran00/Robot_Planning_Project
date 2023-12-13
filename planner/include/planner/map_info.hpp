#ifndef MAP_INFO_HPP_
#define MAP_INFO_HPP_

#include <vector>
#include <tuple>
#include "kdtree.hpp"
#include "rrt.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"

class MapInfo : public rclcpp::Node
{
private:
    enum MarkerID
    {
        _id_boundary = 0,
        _id_obstacle,
        _id_start,
        _id_end,
        _id_path,
        _id_openlist,
        _id_closelist,
        _id_rand_points,
        _id_roadmap,
        _id_rand_point,
        _id_rrt,
    };
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr _marker_pub;
    double _width, _height;
    visualization_msgs::msg::Marker _line_boundary;
    visualization_msgs::msg::Marker _obstacle;
    KDTree _okdtree;
    visualization_msgs::msg::Marker _m_start;
    visualization_msgs::msg::Marker _m_end;
    visualization_msgs::msg::Marker _m_openlist;
    visualization_msgs::msg::Marker _m_closelist;
    visualization_msgs::msg::Marker _m_path;
    visualization_msgs::msg::Marker _m_rand_points;
    visualization_msgs::msg::Marker _m_roadmap;
    visualization_msgs::msg::Marker _m_rand_point;
    visualization_msgs::msg::Marker _m_rrt;
    int _pub_i;

public:
    KDPoint pt_start;
    KDPoint pt_end;
    MapInfo();
    ~MapInfo();

    void set_boundary(int w, int h);
    double get_width(void) { return _width; }
    double get_height(void) { return _height; }
    void set_obstacle(std::vector<KDPoint> &points);
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
