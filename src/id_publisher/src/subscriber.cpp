#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

void topic_callback(const std_msgs::msg::Int32::SharedPtr msg)
{
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "I heard: '%d'", msg->data);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("id_subscriber_node");
  auto subscription = node->create_subscription<std_msgs::msg::Int32>(
    "Balapan", 10, topic_callback);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
