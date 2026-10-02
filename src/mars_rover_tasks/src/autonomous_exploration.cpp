#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <utility>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

class AutonomousExplorationNode : public rclcpp::Node {
private:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
      subscriber_laser_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscriber_odom_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;

  // Initialized data members
  bool turning_{false};
  bool returning_to_origin_{false};
  double turn_direction_{-0.5};
  double distance_from_origin_{0.0};
  double current_position_x_{0.0};
  double current_position_y_{0.0};
  double yaw_{0.0};

public:
  AutonomousExplorationNode() : Node("autonomous_exploration_node") {
    subscriber_laser_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/laser_scan", 10,
        std::bind(&AutonomousExplorationNode::laserscan_callback, this,
                  std::placeholders::_1));

    subscriber_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10,
        std::bind(&AutonomousExplorationNode::odom_callback, this,
                  std::placeholders::_1));

    publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    RCLCPP_INFO(this->get_logger(), "Autonomous Exploration Node Ready...");
  }

private:
  void laserscan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    if (returning_to_origin_) {
      return_to_origin();
      return;
    }

    std::map<std::string, std::pair<int, int>> sectors_laser = {
        {"Right_Rear", {0, 33}},    {"Right", {34, 66}},
        {"Front_Right", {67, 100}}, {"Front_Left", {101, 133}},
        {"Left", {134, 166}},       {"Left_Rear", {167, 199}}};

    std::map<std::string, float> min_distance;

    for (const auto &sector : sectors_laser) {
      int start_idx = sector.second.first;
      int end_idx = sector.second.second;
      float min_val = std::numeric_limits<float>::infinity();

      if (start_idx < static_cast<int>(msg->ranges.size()) &&
          end_idx < static_cast<int>(msg->ranges.size())) {
        for (int i = start_idx; i <= end_idx; ++i) {
          float r = msg->ranges[i];
          if (std::isfinite(r) && r >= msg->range_min && r <= msg->range_max) {
            if (r < min_val) {
              min_val = r;
            }
          }
        }
      }
      min_distance[sector.first] = min_val;
    }

    float obstacle_threshold = 0.8f;
    std::map<std::string, bool> current_detections;
    for (const auto &distance : min_distance) {
      current_detections[distance.first] = distance.second < obstacle_threshold;
    }

    auto apply_action = geometry_msgs::msg::Twist();

    // Priority 1: Front detection
    if (current_detections["Front_Left"] || current_detections["Front_Right"]) {
      if (!turning_) {
        turning_ = true;
        turn_direction_ = -0.5; // Turn right
      }
      apply_action.angular.z = turn_direction_;
      RCLCPP_INFO(this->get_logger(), "Obstacle ahead, turning to clear path.");
    }
    // Priority 2: Side detections
    else {
      turning_ = false;

      if (current_detections["Left"]) {
        apply_action.linear.x = 0.2;
        apply_action.angular.z = -0.3; // Slight right
        RCLCPP_INFO(this->get_logger(),
                    "Obstacle on left, steering slightly right.");
      } else if (current_detections["Right"]) {
        apply_action.linear.x = 0.2;
        apply_action.angular.z = 0.3; // Slight left
        RCLCPP_INFO(this->get_logger(),
                    "Obstacle on right, steering slightly left.");
      }
      // Priority 3: Rear detections
      else if (current_detections["Right_Rear"]) {
        apply_action.linear.x = 0.3;
        RCLCPP_INFO(this->get_logger(),
                    "Obstacle on right rear, moving forward.");
      } else if (current_detections["Left_Rear"]) {
        apply_action.linear.x = 0.3;
        RCLCPP_INFO(this->get_logger(),
                    "Obstacle on left rear, moving forward.");
      } else {
        apply_action.linear.x = 0.5;
        RCLCPP_INFO(this->get_logger(), "No obstacles, moving forward.");
      }
    }

    publisher_->publish(apply_action);
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_position_x_ = msg->pose.pose.position.x;
    current_position_y_ = msg->pose.pose.position.y;

    distance_from_origin_ =
        std::sqrt(current_position_x_ * current_position_x_ +
                  current_position_y_ * current_position_y_);

    tf2::Quaternion q(
        msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch;
    m.getRPY(roll, pitch, yaw_);

    if (distance_from_origin_ > 5.0) {
      returning_to_origin_ = true;
    } else {
      returning_to_origin_ = false;
    }
  }

  void return_to_origin() {
    auto action = geometry_msgs::msg::Twist();

    double desired_yaw = std::atan2(-current_position_y_, -current_position_x_);
    double yaw_error = desired_yaw - yaw_;

    // Normalize yaw error to [-pi, pi]
    yaw_error = std::atan2(std::sin(yaw_error), std::cos(yaw_error));

    if (std::abs(yaw_error) > 0.1) {
      action.angular.z = (yaw_error > 0) ? 0.5 : -0.5;
      RCLCPP_INFO(this->get_logger(), "Turning towards origin. Yaw error: %.2f",
                  yaw_error);
    } else {
      action.linear.x = 0.5;
      RCLCPP_INFO(this->get_logger(), "Heading towards origin.");
    }

    publisher_->publish(action);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AutonomousExplorationNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}