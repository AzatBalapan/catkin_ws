#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

using namespace std::chrono_literals;

class IdPublisherNode : public rclcpp::Node
{
public:
  IdPublisherNode()
  : Node("id_publisher_node"), id_("201769393"), index_(0)
  {
    publisher_ = this->create_publisher<std_msgs::msg::Int32>("Balapan", 10);
    timer_ = this->create_wall_timer(
      10ms, std::bind(&IdPublisherNode::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = std_msgs::msg::Int32();
    message.data = id_[index_] - '0';
    RCLCPP_INFO(this->get_logger(), "Publishing: '%d'", message.data);
    publisher_->publish(message);
    index_ = (index_ + 1) % id_.length();
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
  std::string id_;
  size_t index_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IdPublisherNode>());
  rclcpp::shutdown();
  return 0;
}