#include <iostream>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "planner/voronoi_boost.h"
#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "obstacles_msgs/msg/obstacle_msg.hpp"
#include "obstacles_msgs/msg/obstacle_array_msg.hpp"

#define PRECISION 100 // [cm]

std::vector<Point> circumscribeCircle(double x, double y, double radius, int numSides)
{
    double interiorAngle = 360.0 / numSides;
    std::vector<Point> vertices;

    // Calculate the radius for the circumscribed circle
    double circumscribedRadius = radius / cos(M_PI / numSides);

    for (int i = 0; i < numSides; ++i)
    {
        double angle = i * interiorAngle * M_PI / 180.0; // Convert degrees to radians
        double vertexX = x + circumscribedRadius * cos(angle);
        double vertexY = y + circumscribedRadius * sin(angle);
        vertices.emplace_back(vertexX, vertexY);
    }

    return vertices;
}

using namespace std::chrono_literals;
/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */
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

class WKTGenerator : public rclcpp::Node
{
public:
    WKTGenerator()
        : Node("WKTGenerator")
    {
        const auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_custom);
        subscription_obstacles_ = this->create_subscription<obstacles_msgs::msg::ObstacleArrayMsg>(
            "/obstacles", qos, std::bind(&WKTGenerator::obstacles_cb, this, std::placeholders::_1));
        subscription_borders_ = this->create_subscription<geometry_msgs::msg::PolygonStamped>(
            "/borders", qos, std::bind(&WKTGenerator::borders_cb, this, std::placeholders::_1));
        obstacles_received_ = false;
        borders_received_ = false;
        created_wkt_ = false;
        RCLCPP_INFO(this->get_logger(), "Node started");
    }

private:
    void obstacles_cb(const obstacles_msgs::msg::ObstacleArrayMsg &msg)
    {
        // RCLCPP_INFO(this->get_logger(), "Obstacles received!");
        // store obstacles into class member
        this->obstacles_ = std::move(msg);
        this->obstacles_received_ = true;
        if (this->obstacles_received_ && this->borders_received_ && !this->created_wkt_)
        {
            createWKT();
        }
    }
    void borders_cb(const geometry_msgs::msg::PolygonStamped &msg)
    {
        // RCLCPP_INFO(this->get_logger(), "Map borders received!");
        // store borders into class member
        this->borders_ = std::move(msg);
        this->borders_received_ = true;
        if (this->obstacles_received_ && this->borders_received_ && !this->created_wkt_)
        {
            createWKT();
        }
    }
    void createWKT()
    {
        RCLCPP_INFO(this->get_logger(), "Map borders received!");
        RCLCPP_INFO(this->get_logger(), "Obstacles received!");
        RCLCPP_INFO(this->get_logger(), "Creating received!");

        std::string wkt = "POLYGON((";
        // add borders
        for (int i = 0; i < this->borders_.polygon.points.size(); i++)
        {
            wkt += to_string((int)(this->borders_.polygon.points[i].x * PRECISION)) + " " +
                   to_string((int)(this->borders_.polygon.points[i].y * PRECISION));
            if (i != this->borders_.polygon.points.size() - 1)
            {
                wkt += ", ";
            }
        }
        wkt += ", " + to_string((int)(this->borders_.polygon.points[0].x * PRECISION)) + " " +
                      to_string((int)(this->borders_.polygon.points[0].y * PRECISION));
        wkt += ")";

        // add obstacles
        for (int i = 0; i < this->obstacles_.obstacles.size(); i++)
        {
            if (this->obstacles_.obstacles[i].polygon.points.size() <= 1)
            {
                continue;
            }
            wkt += ", (";
            for (int j = 0; j < this->obstacles_.obstacles[i].polygon.points.size(); j++)
            {
                wkt += to_string((int)(this->obstacles_.obstacles[i].polygon.points[j].x * PRECISION)) + " " +
                       to_string((int)(this->obstacles_.obstacles[i].polygon.points[j].y * PRECISION));
                if (j != this->obstacles_.obstacles[i].polygon.points.size() - 1)
                {
                    wkt += ", ";
                }
            }

            // close polygon
            wkt += ", " + to_string((int)(this->obstacles_.obstacles[i].polygon.points[0].x * PRECISION)) + " " +
                          to_string((int)(this->obstacles_.obstacles[i].polygon.points[0].y * PRECISION));
            wkt += ")";
        }

        wkt += ")";

        // print wkt
        RCLCPP_INFO(this->get_logger(), "WKT: %s", wkt.c_str());

        string save_path = ament_index_cpp::get_package_share_directory("planner") + "/data/polygon.txt";
        std::ofstream fout(save_path);
        fout << wkt;
        fout.close();
        exit(0);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<obstacles_msgs::msg::ObstacleArrayMsg>::SharedPtr subscription_obstacles_;
    rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr subscription_borders_;
    geometry_msgs::msg::PolygonStamped borders_;
    obstacles_msgs::msg::ObstacleArrayMsg obstacles_;
    bool obstacles_received_;
    bool borders_received_;
    bool created_wkt_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WKTGenerator>());
    rclcpp::shutdown();
    return 0;
}