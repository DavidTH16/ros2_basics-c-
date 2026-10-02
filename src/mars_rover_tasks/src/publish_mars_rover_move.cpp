
#include <csignal>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>

class PublisherNode : public rclcpp::Node {
private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

private:
  void publisher_callback() {

    // once the callback is called, create a Twist message to publish
    //  create the msg then init the values
    auto msg = geometry_msgs::msg::Twist();

    msg.linear.x = 0.5;
    msg.angular.z = 0.5;

    // publish the message
    publisher_->publish(msg);
    // display updated info
    RCLCPP_INFO(this->get_logger(), "Publishing: linear.x=%.2f, angular.z=%.2f",
                msg.linear.x, msg.angular.z);
  }

public:
  PublisherNode(const std::string &node_name = "publisher_node")
      : Node(node_name) {
    // create the publisher object to publish on
    // topic/cmd_vel with a queue size of 10 msg.
    publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    // set the timer frequency to publish the msg, 0.5s.
    auto timer_period = std::chrono::milliseconds(500);
    // timer to call the callback
    timer_ = this->create_wall_timer(
        timer_period, std::bind(&PublisherNode::publisher_callback, this));
  }

public:
  // method to stop robot once crtl+z is pressed
  void stop_rover() {
    auto stop_msg = geometry_msgs::msg::Twist(); // init the msg.linear and
                                                 // msg.angulor with zeros
    publisher_->publish(stop_msg);
    RCLCPP_INFO(this->get_logger(), "Publishing stop message before shutdown");
    rclcpp::shutdown();
  }
};

std::shared_ptr<PublisherNode> simple_publisher;

void signal_handler(int signum) { simple_publisher->stop_rover(); }

int main(int argc, char **argv) {

  rclcpp::init(argc, argv);
  simple_publisher = std::make_shared<PublisherNode>();

  // rergister the signal hanlder for CTRL+c
  signal(SIGINT, signal_handler);
  rclcpp::spin(simple_publisher);
  // simple_publisher->stop_rover();
  rclcpp::shutdown();
  return 0;
}
