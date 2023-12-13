#include <iostream>
#include <cmath>
#include <vector>
#include <random>
#include <algorithm>
#include "rclcpp/rclcpp.hpp"

double deg2rad(double deg)
{
    return deg * M_PI / 180.0;
}

class RRT
{
public:
    class Node
    {
    public:
        double x, y;
        std::unique_ptr<Node> parent;

        Node(double x, double y) : x(x), y(y), parent(nullptr) {}
    };

    std::vector<std::unique_ptr<Node>> node_list;

    // Other RRT member functions...
};

// Define any missing utility functions such as deg2rad and dubins_path_planner if not already defined in your C++ code.

class RRTDubins : public RRT
{
public:
    class Node : public RRT::Node
    {
    public:
        double cost;
        double yaw;
        std::vector<double> path_yaw;

        Node(double x, double y, double yaw) : RRT::Node(x, y), cost(0), yaw(yaw) {}
    };

    Node start;
    Node end;
    double min_rand, max_rand;
    int goal_sample_rate;
    int max_iter;
    std::vector<std::vector<double>> obstacle_list;
    double curvature;
    double goal_yaw_th;
    double goal_xy_th;
    double robot_radius;

    RRTDubins(std::vector<double> start, std::vector<double> goal,
              std::vector<std::vector<double>> obstacle_list,
              std::vector<double> rand_area,
              int goal_sample_rate = 10,
              int max_iter = 200,
              double robot_radius = 0.0)
        : start(start[0], start[1], start[2]),
          end(goal[0], goal[1], goal[2]),
          min_rand(rand_area[0]),
          max_rand(rand_area[1]),
          goal_sample_rate(goal_sample_rate),
          max_iter(max_iter),
          obstacle_list(obstacle_list),
          curvature(1.0),
          goal_yaw_th(deg2rad(1.0)),
          goal_xy_th(0.5),
          robot_radius(robot_radius) {}

    // void draw_graph(Node* rnd = nullptr);
    // void plot_start_goal_arrow();
    // Node* steer(Node* from_node, Node* to_node);
    // double calc_new_cost(Node* from_node, Node* to_node);
    // Node* get_random_node();
    // int search_best_goal_node();
    // std::vector<std::vector<double>> generate_final_course(int goal_index);
    // std::vector<std::vector<double>> planning(bool animation = true, bool search_until_max_iter = true);
    std::vector<std::vector<double>> generate_final_course(int goal_index)
    {
        std::cout << "final" << std::endl;
        std::vector<std::vector<double>> path = {{end.x, end.y}};
        // std::unique_ptr<Node> goal = new std::unique_ptr<Node>(node_list[goal_index]);

        while (node_list[goal_index]->parent)
        {
            for (auto it = node_list[goal_index]->rbegin(), it != node_list[goal_index]->rend(); ++it)
            {
                path.push_back({*it->x, *it->y});
            }
            node = dynamic_cast<RRTDubins::Node *>(node->parent);
        }

        path.push_back({start.x, start.y});
        return path;
    }
};

    // Implement the remaining member functions...

    int main()
    {
        std::cout << "Start RRT Dubins Planning" << std::endl;

        // Define obstacleList, start, goal, and other parameters

        std::vector<std::vector<double>> obstacleList = {
            {5, 5, 1},
            {3, 6, 2},
            {3, 8, 2},
            {3, 10, 2},
            {7, 5, 2},
            {9, 5, 2}};

        std::vector<double> start = {0.0, 0.0, deg2rad(0.0)};
        std::vector<double> goal = {10.0, 10.0, deg2rad(0.0)};

        RRTDubins rrt_dubins(start, goal, obstacleList, {-2.0, 15.0});
        // std::vector<std::vector<double>> path = rrt_dubins.planning(true);

        // Draw final path
        // Implement the drawing function and visualization here...

        return 0;
    }