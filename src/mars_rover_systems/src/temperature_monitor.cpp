#include <chrono>
#include <functional>
#include <iostream>
#include <memory>
#include <random>
#include <string>

#include "rclcpp/rclcpp.hpp"

class Temperature : public rclcpp::Node {
private:
  std::string temp_name_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Random generator state kept as member variables
  std::mt19937 gen_;
  std::uniform_real_distribution<double> temp_dist_;

  void monitor_temperature_callback() {
    auto current_temp = random_temperature();
    if (current_temp < 70.0) {
      RCLCPP_INFO(this->get_logger(), "[%s] Current temperature: %.2f C",
                  temp_name_.c_str(), current_temp);
    } else {
      RCLCPP_WARN(this->get_logger(),
                  "[%s] Warning: High temperature detected! %.2f C",
                  temp_name_.c_str(), current_temp);
    }
  }

  double random_temperature() { return temp_dist_(gen_); }

public:

   // initi the object constructor
  Temperature(const std::string &node_name, double timer_period = 0.2)
      : Node(node_name), temp_name_(node_name), gen_(std::random_device{}()),
        temp_dist_(20.0, 100.0) {

    RCLCPP_INFO(this->get_logger(),
                "Temperature Monitor Node has been started");

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(timer_period * 1000)),
        std::bind(&Temperature::monitor_temperature_callback, this));
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Temperature>("temperature_node", 1.0);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}