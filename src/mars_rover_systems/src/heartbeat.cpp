#include "rclcpp/executors.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/timer.hpp"
#include <chrono>
#include <iostream>
#include <string>

class Heartbeat : public rclcpp::Node {

private:
  std::string rover_name_;
  rclcpp::TimerBase::SharedPtr timer_;

public:
  Heartbeat(const std::string &rover_name, double timer_period = 0.2)
      : Node(rover_name), rover_name_(rover_name) {
    // create a timer sending two parameters
    // the duration between two callbacks (timer_period seconds)
    // the timer function (timer_callback)
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(timer_period * 1000)),
        std::bind(&Heartbeat::timer_callback, this));
  }

private:
  // define callback
  void timer_callback() {
    auto ros_time_stamp = this->get_clock()->now();
    RCLCPP_INFO(this->get_logger(),
                "%s is alive...Time(nanoseconds=%ld, clock_type = ROS_TIME)",
                rover_name_.c_str(), ros_time_stamp.nanoseconds());
  }
};

int main(int argc, char **argv) {

  // init ros communication
  rclcpp::init(argc, argv);
  // declare the node constructor
  auto node = std::make_shared<Heartbeat>("mars_rover_1", 1.0);
  // keeps the node alive, wait for a equest to kill the node (ctrl + c)
  rclcpp::spin(node);
  // shutdown the ros communication
  rclcpp::shutdown();
  return 0;
}