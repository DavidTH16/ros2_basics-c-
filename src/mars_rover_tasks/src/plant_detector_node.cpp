#include "mars_rover_tasks/plant_detector.hpp" // Include the AI class
#include "nav_msgs/msg/detail/odometry__struct.hpp"
#include "rclcpp/subscription.hpp"
#include <cstddef>
#include <custom_interfaces/msg/rover_events.hpp>
#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/msg/odometry.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp> // Include String message type

class PlantDetectorNode : public rclcpp::Node {
private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::Publisher<custom_interfaces::msg::RoverEvents>::SharedPtr publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  std::unique_ptr<PlantDetector> plant_detector_;
  nav_msgs::msg::Odometry::SharedPtr current_odom_;

public:
  PlantDetectorNode() : Node("plant_detector_node") {
    // Initialize the PlantDetector
    std::string path_to_model = "/home/user/ros2_ws/src/basic_ros2_extra_files/"
                                "plant_detector/best_plant_detector_model.pth";
    plant_detector_ = std::make_unique<PlantDetector>(path_to_model);

    // Subscribe to the image topic
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/leo/camera/image_raw", 10,
        std::bind(&PlantDetectorNode::image_callback, this,
                  std::placeholders::_1));

    // Subscribe to the odometry topic
    odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10,
        std::bind(&PlantDetectorNode::odom_callback, this,
                  std::placeholders::_1));

    // Initialize the Publisher for plant detection results in the custom
    // interface
    publisher_ = this->create_publisher<custom_interfaces::msg::RoverEvents>(
        "/mars_rover_events", 10);

    current_odom_ = nullptr;
  }

private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_odom_ = msg;
  }

  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    // Convert ROS Image message to OpenCV image
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception &e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    // Convert BGR to RGB
    cv::Mat rgb_image;
    cv::cvtColor(cv_ptr->image, rgb_image, cv::COLOR_BGR2RGB);

    // Use the PlantDetector to make a prediction
    float prediction = plant_detector_->predict(rgb_image);

    // create a RoverEvents message
    auto rover_event = custom_interfaces::msg::RoverEvents();

    // Determine the result message based on the prediction
    std::string result;
    if (prediction > 0.5) {
      rover_event.info.data =
          "Plant detected with confidence: " + std::to_string(prediction);
      RCLCPP_WARN(this->get_logger(), "%s", rover_event.info.data.c_str());
      RCLCPP_WARN(this->get_logger(), "Publishing mars rover event...");

      if (current_odom_) {
        rover_event.rover_location =
            current_odom_->pose.pose; // copy the pose data from odometry
      }

      // publish the roverevents message
      publisher_->publish(rover_event);

    }

    else {
      rover_event.info.data =
          "No plant detected. Confidence: " + std::to_string(1.0 - prediction);
      RCLCPP_INFO(this->get_logger(), "%s", rover_event.info.data.c_str());
    }
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto plant_detector_node = std::make_shared<PlantDetectorNode>();

  rclcpp::spin(plant_detector_node);

  // Shutdown explicitly
  rclcpp::shutdown();
  return 0;
}