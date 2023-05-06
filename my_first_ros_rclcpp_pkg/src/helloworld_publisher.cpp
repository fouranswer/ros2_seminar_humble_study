#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;
// 추후에 500ms, 1s과 같이 시간을 가식성을 높인 문자로 표현하기 위하여 namespace를 사용할 수 있도록 선언하였다.
// namespace 이름으로 묶는 기능, 함수이름이 같아도 다른 namespace이면 상관없다.

class HelloworldPublisher : public rclcpp::Node
{
public:
  HelloworldPublisher()
  : Node("helloworld_publisher"), count_(0)    // 생성 및 초기화
  {
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));  // auto: 타입형은 알아서 설정
    helloworld_publisher_ = this->create_publisher<std_msgs::msg::String>(
      "helloworld", qos_profile);
    timer_ = this->create_wall_timer(
      1s, std::bind(&HelloworldPublisher::publish_helloworld_msg, this));

    // lambda 표현
    // timer_ = this->create_wall_timer(
    //   1s,
    //   [this]() -> void
    //     {
    //       auto msg = std_msgs::msg::String();
    //       msg.data = "Hello World2: " + std::to_string(count_++);
    //       RCLCPP_INFO(this->get_logger(), "Published message: '%s'", msg.data.c_str());
    //       helloworld_publisher_->publish(msg);
    //     }
    //   );
  }

private:
  void publish_helloworld_msg()
  {
    auto msg = std_msgs::msg::String();
    msg.data = "Hello World: " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Published message: '%s'", msg.data.c_str());
    helloworld_publisher_->publish(msg);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr helloworld_publisher_;  // SharedPtr 스마트 포인터로서 메모리 해제를 자동으로 해준다.
  size_t count_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<HelloworldPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
