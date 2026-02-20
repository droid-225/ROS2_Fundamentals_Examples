/*
Description:
  This is a ROS 2 node subscribes to "Hello World" messages

------
Publishing Topics:
  None

Subscription Topics:
  The channel containing the "Hello World" messages
  /cpp_example_topic - std_msgs/String
------
Author: Rishit Shah
Date: February 20, 2026
*/

#include "rclcpp/rclcpp.hpp" // ROS 2 C++ client library
#include "std_msgs/msg/string.hpp" // Handling String messages

using std::placeholders::_1; // Placeholder for callback function

class MinimalCppSubscriber : public rclcpp::Node
{
  public:
    MinimalCppSubscriber() : Node("minimal_cpp_subscriber")
    {
      subscriber_ = create_subscription<std_msgs::msg::String> (
        "/cpp_example_topic",
        10,
        std::bind(
          &MinimalCppSubscriber::topicCallback,
          this,
          _1
        )
      );
    }

    void topicCallback(const std_msgs::msg::String & msg) const {
      RCLCPP_INFO_STREAM(get_logger(), "I heard: " << msg.data.c_str());
    }

  private:
    // Member variables
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);

  auto minimal_cpp_subscriber_node = std::make_shared<MinimalCppSubscriber>();
  rclcpp::spin(minimal_cpp_subscriber_node);

  rclcpp::shutdown();

  return 0;
}
