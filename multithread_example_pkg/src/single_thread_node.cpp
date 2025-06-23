#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("single_thread_node");

  auto pub = node->create_publisher<std_msgs::msg::String>("topic", 10);
  auto timer = node->create_wall_timer(
      std::chrono::milliseconds(500),
      [pub]()
      {
        auto msg = std_msgs::msg::String();
        msg.data = "Hello SingleThreadedExecutor";
        pub->publish(msg);
      });

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
