#include <memory>
#include <chrono>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class MultiThreadNode : public rclcpp::Node
{
public:
    MultiThreadNode(bool use_multithread)
        : Node("multi_thread_node"),
          use_multithread_(use_multithread)
    {
        // Callback Group 생성
        exclusive_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);  // 한 번에 하나의 콜백만 실행
        reentrant_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::Reentrant);          // 여러 콜백 동시 실행 가능

        // 블로킹 타이머 생성 (Exclusive 그룹)
        blocking_timer_ = this->create_wall_timer(
            1000ms, // 1초 주기
            [this]()
            { this->blockingCallback(); },
            exclusive_group_);
        RCLCPP_INFO(this->get_logger(), "블로킹 타이머 생성 완료 (1초 주기)");

        // 퍼블리시 타이머 생성 (Reentrant 그룹)
        publish_timer_ = this->create_wall_timer(
            250ms, // 250ms 주기
            [this]()
            { this->publishCallback(); },
            reentrant_group_);
        RCLCPP_INFO(this->get_logger(), "퍼블리시 타이머 생성 완료 (250ms 주기)");

        // 퍼블리셔 생성 (Reentrant 그룹)
        rclcpp::PublisherOptions pub_options;
        pub_options.callback_group = reentrant_group_;
        publisher_ = this->create_publisher<std_msgs::msg::String>(
            "/topic", 10, pub_options);
        RCLCPP_INFO(this->get_logger(), "퍼블리셔 생성 완료");

        RCLCPP_INFO(this->get_logger(),
                    "노드 초기화 완료 (%s Executor 사용)",
                    use_multithread_ ? "멀티스레드" : "싱글스레드");
    }

private:
    void blockingCallback()
    {
        RCLCPP_INFO(get_logger(), "[Blocking] 2초 블로킹 시작 '%d'", count_b++);
        // 2초간 블로킹 (의도적 지연)
        std::this_thread::sleep_for(2000ms);
        RCLCPP_INFO(get_logger(),"[Blocking] 블로킹 완료");
    }

    void publishCallback()
    {
        auto msg = std_msgs::msg::String();
        msg.data = "count: " + std::to_string(count++);
        publisher_->publish(msg);
        RCLCPP_INFO(get_logger(),
                    "[Publish] 메시지 발행: '%s'", msg.data.c_str());
    }

    // 멤버 변수
    bool use_multithread_;
    rclcpp::CallbackGroup::SharedPtr exclusive_group_;
    rclcpp::CallbackGroup::SharedPtr reentrant_group_;
    rclcpp::TimerBase::SharedPtr blocking_timer_;
    rclcpp::TimerBase::SharedPtr publish_timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    int count = 0;
    int count_b = 0;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    bool use_multithread = (argc > 1 && std::string(argv[1]) == "multi");
    
    // Executor 설정
    std::shared_ptr<rclcpp::Executor> executor;
    if (use_multithread)
        // 멀티스레드 Executor (4개 스레드)
        executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>(
            rclcpp::ExecutorOptions(), 4);
    else
        executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

    // 노드 생성 및 실행
    auto node = std::make_shared<MultiThreadNode>(use_multithread);
    executor->add_node(node);
    RCLCPP_INFO(node->get_logger(), "Executor spin 시작");
    executor->spin();

    rclcpp::shutdown();
    return 0;
}
