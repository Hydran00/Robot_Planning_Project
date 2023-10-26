#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <unistd.h>

#include <iostream>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"

#include "graph_msgs/msg/geometry_graph.hpp"
#include "graph_msgs/msg/edges.hpp"

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/named_function_params.hpp>
#include <boost/array.hpp>
#include <array>
#include <utility>

class  Evader : public rclcpp::Node
{
  public:
     Evader()
    : Node("evader")
    {
        const auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);

        evader_tf = this->create_subscription<geometry_msgs::msg::TransformStamped>(
        "shelfino1/transform", qos, std::bind(& Evader::handle_transform1, this, std::placeholders::_1));
        pursuer_tf = this->create_subscription<geometry_msgs::msg::TransformStamped>(
        "shelfino2/transform", qos, std::bind(& Evader::handle_transform2, this, std::placeholders::_1));

        sub_graph = this->create_subscription<graph_msgs::msg::GeometryGraph>(
        "roadmap", qos, std::bind(& Evader::handle_roadmap, this, std::placeholders::_1));
    }

  
  private:

    void handle_transform1(const std::shared_ptr<geometry_msgs::msg::TransformStamped> msg)
    {
        

        return;
    }

    void handle_transform2(const std::shared_ptr<geometry_msgs::msg::TransformStamped> msg)
    {
        

        return;
    }

    void handle_roadmap(const std::shared_ptr<graph_msgs::msg::GeometryGraph> msg)
    {
        typedef boost::adjacency_list<boost::listS, boost::vecS,
            boost::undirectedS, boost::no_property,
            boost::property<boost::edge_weight_t, int>> graph;

        points = msg->nodes;

        int count = 0;
        for (int i = 0; i < msg->edges.size(); i++) {
            for (auto j = msg->edges.at(i).node_ids.begin(); j != msg->edges.at(i).node_ids.end(); j++) {
                edges.push_back(std::make_pair(i, *j));
                weights.push_back(1.0);
                count++;
            }
        }

        graph g{edges.begin(), edges.end(), weights.begin(), count};

        // boost::array<int, 4> directions;
        // boost::dijkstra_shortest_paths(g, bottomRight,
        //     boost::predecessor_map(directions.begin()));

        // int p = topLeft;
        // while (p != bottomRight)
        // {
        //     std::cout << p << '\n';
        //     p = directions[p];
        // }
        // std::cout << p << '\n';

        // roadmap_ready = true;
        return;
    }

    bool roadmap_ready = false;
    std::vector<geometry_msgs::msg::Point> points;
    std::vector<std::pair<int, int>> edges;
    std::vector<double> weights;
    

    rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr evader_tf;
    rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr pursuer_tf;
    rclcpp::Subscription<graph_msgs::msg::GeometryGraph>::SharedPtr sub_graph;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared< Evader>());
  rclcpp::shutdown();
  return 0;
}