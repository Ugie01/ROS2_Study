#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <thread>

class MultiNode : public rclcpp::Node
{
public:
  MultiNode() : Node("multi_thread_node")
  {
    pub_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(300),
        [this]()
        {
          auto msg = std_msgs::msg::String();
          msg.data = "Hello MultiThreadedExecutor";
          pub_->publish(msg);
        });
    // 긴 작업 시뮬레이션
    sub_ = this->create_subscription<std_msgs::msg::String>(
        "topic", 10,
        [this](std_msgs::msg::String::SharedPtr msg)
        {
          RCLCPP_INFO(this->get_logger(), "Received: '%s' on thread %zu",
                      msg->data.c_str(), std::hash<std::thread::id>{}(std::this_thread::get_id()));
          std::this_thread::sleep_for(std::chrono::milliseconds(200));
        });
  }

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MultiNode>();

  rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 4);
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
