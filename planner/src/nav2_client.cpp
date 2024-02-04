#include <unistd.h>

#include <chrono>
#include <functional>
#include <iostream>
#include <memory>
#include <string>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"

using FollowPath = nav2_msgs::action::FollowPath;

// typedef struct RobotPosition {
//   float x = 0;
//   float y = 0;
//   float theta = 0;
//   bool is_updated = false;
// };

// typedef struct RPY {
//   float roll;
//   float pitch;
//   float yaw;
//   RPY(float R, float P, float Y) {
//     roll = R;
//     pitch = P;
//     yaw = Y;
//   }
//   RPY() {
//     roll = 0;
//     pitch = 0;
//     yaw = 0;
//   }
//   tf2::Quaternion getQuaternion() {
//     tf2::Quaternion q;
//     q.setRPY(roll, pitch, yaw);
//     return q;
//   }
// };

// RPY fromQuaternion(tf2::Quaternion q) {
//   tf2::Matrix3x3 m(q);
//   double roll, pitch, yaw;
//   m.getRPY(roll, pitch, yaw);
//   return RPY((float)roll, (float)pitch, (float)yaw);
// }

class PathPublisher : public rclcpp::Node {
 public:
  PathPublisher() : Node("Nav2Client") {
    client_ptr_ =
        rclcpp_action::create_client<FollowPath>(this, "shelfino0/follow_path");

    _path_subscription = this->create_subscription<nav_msgs::msg::Path>(
        "shelfino0/plan1", 10,
        std::bind(&PathPublisher::store_path, this, std::placeholders::_1));

    if (!this->client_ptr_->wait_for_action_server()) {
      RCLCPP_ERROR(this->get_logger(),
                   "Action server not available after waiting");
      rclcpp::shutdown();
    }
    std::cout << "Action client ready" << std::endl;
  }
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr _path_subscription;
  geometry_msgs::msg::TransformStamped t;

  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr subscription1;
  rclcpp_action::Client<FollowPath>::SharedPtr client_ptr_;
  // geometry_msgs::msg::PoseArray waypoints;
  nav_msgs::msg::Path full_path;
  // RobotPosition robot_pose;
  bool waypoints_received = false;
  bool already_received = false;

  void store_path(const nav_msgs::msg::Path& msg) {
    if (already_received) {
      return;
    }
    std::cout << "Storing path" << std::endl;
    full_path = msg;
    waypoints_received = true;
    already_received = true;
    return;
  }

  // void follow_path(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
  //   if (!robot_pose.is_updated) {
  //     RCLCPP_INFO(this->get_logger(), "Cannot plan: Robot position unknown");
  //     return;
  //   }
  //   store_path(msg);
  //   return;
  // }
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PathPublisher>();
  while (rclcpp::ok()) {
    if (node->waypoints_received) {
      std::cout << "\033[1;32mSending path to controller\033[0m" << std::endl;
      // nav_msgs::msg::Path full_path;
      node->full_path.header.stamp = node->get_clock()->now();
      node->full_path.header.frame_id = "map";
      std::cout << "1" << std::endl;

      auto goal_msg = FollowPath::Goal();
      goal_msg.path = node->full_path;
      for (size_t i = 0; i < goal_msg.path.poses.size(); i++) {
        std::cout << "Position: " << goal_msg.path.poses[i].pose.position.x
                  << ", " << goal_msg.path.poses[i].pose.position.y
                  << std::endl;
        std::cout << "Orientation: "
                  << goal_msg.path.poses[i].pose.orientation.x << ", "
                  << goal_msg.path.poses[i].pose.orientation.y << ", "
                  << goal_msg.path.poses[i].pose.orientation.z << ", "
                  << goal_msg.path.poses[i].pose.orientation.w << std::endl;
        std::cout << "---------------------" << std::endl;
      }
      goal_msg.controller_id = "FollowPath";

      std::cout << "2" << std::endl;

      node->client_ptr_->async_send_goal(goal_msg);

      node->waypoints_received = false;
      std::cout << "3" << std::endl;
    }
    rclcpp::spin_some(node);
  }

  rclcpp::shutdown();

  return 0;
}
