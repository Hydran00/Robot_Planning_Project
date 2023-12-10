#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <unistd.h>

#include <iostream>

#include "dubins.cpp"
#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/pose.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include "nav2_msgs/action/follow_path.hpp"

#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

using FollowPath = nav2_msgs::action::FollowPath;

typedef struct RobotPosition
{
    float x = 0;
    float y = 0;
    float theta = 0;
    bool is_updated = false;
};
typedef struct RPY
{
    float roll;
    float pitch;
    float yaw;
    RPY(float R, float P, float Y)
    {
        roll = R;
        pitch = P;
        yaw = Y;
    }
    RPY()
    {
        roll = 0;
        pitch = 0;
        yaw = 0;
    }
    tf2::Quaternion getQuaternion()
    {
        tf2::Quaternion q;
        q.setRPY(roll, pitch, yaw);
        return q;
    }
};
RPY fromQuaternion(tf2::Quaternion q)
{
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return RPY((float)roll, (float)pitch, (float)yaw);
}

class PathPublisher : public rclcpp::Node
{
public:
    PathPublisher()
        : Node("path_talker")
    {
        const auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);
        subscription_1 = this->create_subscription<geometry_msgs::msg::TransformStamped>(
            "transform", qos, std::bind(&PathPublisher::store_current_transform, this, std::placeholders::_1));
        subscription_2 = this->create_subscription<geometry_msgs::msg::PoseArray>(
            "voronoi_waypoints", qos, std::bind(&PathPublisher::plan_dubins, this, std::placeholders::_1));

        publisher_ = this->create_publisher<nav_msgs::msg::Path>("plan", 10);

        client_ptr_ = rclcpp_action::create_client<FollowPath>(this, "follow_path");

        RCLCPP_INFO(this->get_logger(), "Node started");
    }
    geometry_msgs::msg::TransformStamped t;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher_;
    rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr subscription_1;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr subscription_2;
    rclcpp_action::Client<FollowPath>::SharedPtr client_ptr_;
    geometry_msgs::msg::PoseArray waypoints;
    RobotPosition robot_pose;
    bool waypoints_received = false;
    bool already_received = false;

    void store_waypoints(const geometry_msgs::msg::PoseArray::SharedPtr msg)
    {
        if (already_received)
        {
            return;
        }
        geometry_msgs::msg::Pose pose;
        for (int i = 0; i < msg->poses.size(); i++)
        {
            pose.position.x = msg->poses[i].position.x;
            pose.position.y = msg->poses[i].position.y;
            pose.orientation.x = msg->poses[i].orientation.x;
            pose.orientation.y = msg->poses[i].orientation.y;
            pose.orientation.z = msg->poses[i].orientation.z;
            pose.orientation.w = msg->poses[i].orientation.w;
            waypoints.poses.push_back(pose);
        }
        waypoints_received = true;
        already_received = true;
        RCLCPP_INFO(this->get_logger(), "Waypoints stored");
        return;
    }

    void store_current_transform(const std::shared_ptr<geometry_msgs::msg::TransformStamped> msg)
    {
        robot_pose.x = msg->transform.translation.x;
        robot_pose.y = msg->transform.translation.y;
        tf2::Quaternion q(msg->transform.rotation.x, msg->transform.rotation.y, msg->transform.rotation.z, msg->transform.rotation.w);
        // tf2::Matrix3x3 m(q);
        RPY rpy = fromQuaternion(q);
        robot_pose.theta = rpy.yaw;
        robot_pose.is_updated = true;
        // RCLCPP_INFO(this->get_logger(), "Current position stored: x: %f, y: %f, yaw: %f ", robot_pose.x, robot_pose.y, robot_pose.theta);
        return;
    }

    void plan_dubins(const geometry_msgs::msg::PoseArray::SharedPtr msg)
    {
        // RCLCPP_INFO(this->get_logger(), "msg received");

        if (!robot_pose.is_updated)
        {
            RCLCPP_INFO(this->get_logger(), "Cannot plan: Robot position unknown");
            return;
        }
        store_waypoints(msg);

        return;
    }

    Plot_arc_struct
    plot_arc(Dubins_arc arc)
    {

        float pts[npts + 1][2] = {};
        for (int j = 0; j <= npts; j++)
        {
            float s = arc.L / npts * j;
            Circle_line ci;
            ci = circline(s, arc.x0, arc.y0, arc.th0, arc.k);
            float x = ci.x;
            float y = ci.y;
            pts[j][0] = x;
            pts[j][1] = y;
        }

        Plot_arc_struct a;
        for (int r = 0; r < npts; r++)
        {
            for (int c = 0; c < 2; c++)
            {
                a.pts[r][c] = pts[r][c];
            }
        }
        return a;
    }

    nav_msgs::msg::Path plot_dubins(Dubins_curve curve)
    {
        float ret[((npts + 1) * 3)][2];
        Plot_arc_struct a1 = plot_arc(curve.a1);
        for (int r = 0; r < npts; r++)
        {
            for (int c = 0; c < 2; c++)
            {
                cout << a1.pts[r][c] << "\t";

                ret[r][c] = a1.pts[r][c];
            }
            cout << "\n\n";
        }

        Plot_arc_struct a2 = plot_arc(curve.a2);
        for (int r = 0; r < npts; r++)
        {
            for (int c = 0; c < 2; c++)
            {
                cout << a2.pts[r][c] << "\t";
                ret[r + 100][c] = a2.pts[r][c];
            }
            cout << "\n\n";
        }
        Plot_arc_struct a3 = plot_arc(curve.a3);
        for (int r = 0; r < npts; r++)
        {
            for (int c = 0; c < 2; c++)
            {
                cout << a3.pts[r][c] << "\t";
                ret[r + 200][c] = a3.pts[r][c];
            }
            cout << "\n\n";
        }
        cout << "******************";
        for (int r = 0; r < (npts) * 3; r++)
        {
            for (int c = 0; c < 2; c++)
            {
                cout << ret[r][c] << "\t";
            }
            cout << "\n";
        }

        nav_msgs::msg::Path path_msg;
        std::vector<geometry_msgs::msg::PoseStamped> poses_temp;
        path_msg.header.stamp = this->get_clock()->now();
        path_msg.header.frame_id = "map";
        geometry_msgs::msg::Pose pose_temp;
        geometry_msgs::msg::Point position_temp;
        geometry_msgs::msg::Quaternion quaternion_temp;
        geometry_msgs::msg::PoseStamped pose_stamped_temp;

        for (int i = 0; i < ((npts) * 3); i++)
        {

            position_temp.x = ret[i][0];

            position_temp.y = ret[i][1];
            position_temp.z = 0;

            quaternion_temp.x = 0.0;
            quaternion_temp.y = 0.0;
            quaternion_temp.z = 0.0;
            quaternion_temp.w = 0.0;

            if (i == ((npts) * 3) - 1)
            {
                quaternion_temp.z = 0.9252115;
                quaternion_temp.w = -0.3794518;
            }

            pose_temp.position = position_temp;
            pose_temp.orientation = quaternion_temp;

            pose_stamped_temp.pose = pose_temp;
            pose_stamped_temp.header.stamp = this->get_clock()->now();
            pose_stamped_temp.header.frame_id = "";

            if (i < ((npts) * 3) - 30 || i == ((npts) * 3) - 1)
            {
                poses_temp.push_back(pose_stamped_temp);
            }
        }
        path_msg.poses = poses_temp;

        return path_msg;
    }

    nav_msgs::msg::Path get_dubins_path(geometry_msgs::msg::Pose start, geometry_msgs::msg::Pose end, float Kmax)
    {
        Dubins_curve curve;
        // RCLCPP_INFO(this->get_logger(), "Quat: %f %f %f %f", start.orientation.x, start.orientation.y, start.orientation.z, start.orientation.w);
        RPY start_rpy =fromQuaternion(tf2::Quaternion(start.orientation.x, start.orientation.y, start.orientation.z, start.orientation.w));
        RPY end_rpy = fromQuaternion(tf2::Quaternion(end.orientation.x, end.orientation.y, end.orientation.z, end.orientation.w));
        curve = dubins_shortest_path(start.position.x, start.position.y, start_rpy.yaw, end.position.x, end.position.y, end_rpy.yaw, Kmax);

        RCLCPP_INFO(this->get_logger(), "Going from x: %f, y: %f, yaw: %f to x: %f, y: %f, yaw: %f ",
                    start.position.x, start.position.y, start_rpy.yaw, end.position.x, end.position.y, end_rpy.yaw);
        nav_msgs::msg::Path path_msg = this->plot_dubins(curve);
        return path_msg;
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PathPublisher>();
    while (rclcpp::ok())
    {
        if (node->waypoints_received)
        {
            // if new_m
            nav_msgs::msg::Path full_path;
            full_path.header.stamp = node->get_clock()->now();
            full_path.header.frame_id = "map";

            float Kmax = 1.0;
            geometry_msgs::msg::Pose starting_pose;
            starting_pose.position.x = node->robot_pose.x;
            starting_pose.position.y = node->robot_pose.y;
            starting_pose.position.z = 0;
            // RPY starting_rpy = RPY(0, 0, node->robot_pose.theta);
            // tf2::Quaternion q = starting_rpy.getQuaternion();
            tf2::Quaternion q;
            q.setRPY(0, 0, node->robot_pose.theta);
            starting_pose.orientation.x = q.x();
            starting_pose.orientation.y = q.y();
            starting_pose.orientation.z = q.z();
            starting_pose.orientation.w = q.w();

            // insert the robot's current position as the first waypoint for the first dubins path
            nav_msgs::msg::Path segment = node->get_dubins_path(starting_pose, node->waypoints.poses[0], Kmax);
            full_path.poses.insert(full_path.poses.end(), segment.poses.begin(), segment.poses.end());

            // compute and insert the other dubins paths
            for (int i = 0; i < node->waypoints.poses.size() - 1; i++)
            {
                nav_msgs::msg::Path path_msg = node->get_dubins_path(node->waypoints.poses[i], node->waypoints.poses[i + 1], Kmax);
                full_path.poses.insert(full_path.poses.end(), path_msg.poses.begin(), path_msg.poses.end());
            }

            std::vector<geometry_msgs::msg::PoseStamped> poses_temp;

            if (!node->client_ptr_->wait_for_action_server())
            {
                RCLCPP_ERROR(node->get_logger(), "Action server not available after waiting");
                rclcpp::shutdown();
            }

            node->publisher_->publish(full_path);
            // sleep(0.5);
            // node->publisher_->publish(full_path);
            auto goal_msg = FollowPath::Goal();
            goal_msg.path = full_path;
            goal_msg.controller_id = "FollowPath";

            // RCLCPP_INFO(node->get_logger(), "Sending goal");
            // node->client_ptr_->async_send_goal(goal_msg);
            // sleep(0.5);
            node->client_ptr_->async_send_goal(goal_msg);

            node->waypoints_received = false;
        }

        rclcpp::spin_some(node);
    }
    rclcpp::shutdown();
    return 0;
}