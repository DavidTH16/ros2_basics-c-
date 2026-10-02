#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <utility>

#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/detail/string__struct.hpp"
#include "std_msgs/msg/string.hpp"
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

class AutonomousExplorationNode : public rclcpp::Node {
private:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
      subscriber_laser_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mission_subscriber_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscriber_odom_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_goal;

  // Initialized data members
  double turn_direction_{0.0};
  double current_yaw_{0.0};

  // variables control mission
  bool mission_active_ = false;
  bool has_a_mission = false;

  // varible to specify target, home, and current location
  struct Coordinates {
    double x;
    double y;
  };

  Coordinates home_position_{0.0, 0.0};
  Coordinates pickup_position_{-2.342, -2.442};
  Coordinates target_position{0.0, 0.0};
  Coordinates current_position{0.0, 0.0};

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

    mission_subscriber_ = this->create_subscription<std_msgs::msg::String>(
        "/nasa_mission", 10,
        std::bind(&AutonomousExplorationNode::mission_callback, this,
                  std::placeholders::_1));

    publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    publisher_goal =
        this->create_publisher<std_msgs::msg::String>("/mars_rover_status", 10);

    RCLCPP_INFO(this->get_logger(), "Autonomous Exploration Node Ready...");
  }

private:
  // laser scan callback
  // ----------------------------------------------------------------------------------------
  void laserscan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {

    // check if mission is active
    if (!mission_active_) {
      stop_rover();
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
      turn_direction_ = -0.5; // Turn right
      apply_action.angular.z = turn_direction_;
      RCLCPP_INFO(this->get_logger(), "Obstacle ahead, turning to clear path.");
    }
    // Priority 2: Side detections
    else {

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
        apply_action.linear.x = 0.0; // 0.5
        RCLCPP_INFO(this->get_logger(), "No obstacles, moving forward.");
      }
    }

    publisher_->publish(apply_action);
  }

  // method to check where the robot
  // is-----------------------------------------------------------------
  bool is_at_target(const Coordinates &target) {
    double distance_from_target = calculate_distance(target, current_position);
    RCLCPP_INFO(this->get_logger(), "Distance from target: %.2f",
                distance_from_target);
    return distance_from_target < 0.1;
  }
  // mission
  // callback------------------------------------------------------------------------------------
  void mission_callback(const std_msgs::msg::String::SharedPtr msg) {
    auto message = msg->data;

    if (message == "Go-Home") {
      if (!is_at_target(home_position_)) {
        RCLCPP_INFO(this->get_logger(),
                    "Nasa Command: Go-Home. Returning to base...");
        mission_active_ = true;
        has_a_mission = true;
        target_position = home_position_;
      }
    } else if (message == "Go-Pickup") {
      if (!is_at_target(pickup_position_)) {
        RCLCPP_INFO(this->get_logger(),
                    "Nasa Command: Go-Pickup. Moving to sample site...");
        mission_active_ = true;
        has_a_mission = true;
        target_position = pickup_position_;
      }
    }
  }

  // callback for
  // odometry-----------------------------------------------------------------------------------------------------
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // update the current positon
    current_position.x = msg->pose.pose.position.x;
    current_position.y = msg->pose.pose.position.y;

    // extract yaw from quaternion
    tf2::Quaternion q(
        msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch;
    m.getRPY(roll, pitch, current_yaw_);

    // check if the goal has been reached
    if (has_a_mission) {
      if (mission_active_) {
        double distance_to_goal_ =
            calculate_distance(target_position, current_position);
        RCLCPP_INFO(this->get_logger(), "Distance to the target: %.2f",
                    distance_to_goal_);

        if (distance_to_goal_ < 0.1) // margin of error less 0.1 m
        {
          stop_rover();
          mission_active_ = false;
          RCLCPP_INFO(this->get_logger(), "Goal reached. Mars rover stopped");
        }
      }
    } else {
      return;
    }
  }

  // method to get distance from origin to target, first turn the robot
  // to desired
  // yaw.------------------------------------------------------------------------------
  double calculate_distance(const Coordinates &target,
                            const Coordinates &current_pos) {

    double dx = target.x - current_pos.x;
    double dy = target.y - current_pos.y;

    double distance_to_target = std::hypot(dx, dy);

    auto action = geometry_msgs::msg::Twist();

    double desired_yaw = std::atan2(dy, dx);
    double yaw_error = desired_yaw - current_yaw_;

    // Normalize yaw error to [-pi, pi]
    yaw_error = std::atan2(std::sin(yaw_error), std::cos(yaw_error));

    if (has_a_mission) {
      if (std::abs(yaw_error) > 0.08) {
        action.angular.z = (yaw_error > 0) ? 0.5 : -0.5;
        RCLCPP_INFO(this->get_logger(),
                    "Turning towards target. Yaw error: %.2f", yaw_error);
      } else {
        action.linear.x = 0.5;
        action.angular.z = 0.0;
      }

      publisher_->publish(action);
    }
    return distance_to_target;
  }
  // method to stop the rover once the target has been
  // reached.--------------------------------------------------------------------------------------------------------------
  void stop_rover() {
    geometry_msgs::msg::Twist stop_cmd;
    stop_cmd.linear.x = 0.0;
    stop_cmd.linear.y = 0.0;
    stop_cmd.angular.z = 0.0;
    mission_active_ = false;
    has_a_mission = false;
    publisher_->publish(stop_cmd);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AutonomousExplorationNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}