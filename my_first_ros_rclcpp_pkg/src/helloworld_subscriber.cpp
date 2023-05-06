#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;
// placeholders 클래스는 bind 함수의 대체자 역할을 위하여 _1로 선언하였다.
// https://en.cppreference.com/w/cpp/utility/functional/placeholders

class HelloworldSubscriber : public rclcpp::Node    // rclcpp에서 Node 클래스를 상속받는다.
{
public:
  HelloworldSubscriber()
  : Node("Helloworld_subscriber")
  {
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));   // 마지막 10개까지는 보존한다. qos 참조
    helloworld_subscriber_ = this->create_subscription<std_msgs::msg::String>(
      "helloworld",
      qos_profile,
      std::bind(&HelloworldSubscriber::subscribe_topic_message, this, _1));   // 이벤트와 함수를 연결시켜주는 bind
  }

private:
  void subscribe_topic_message(const std_msgs::msg::String::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "Received message: '%s'", msg->data.c_str());
  }
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr helloworld_subscriber_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);    // rclcpp 초기화
  auto node = std::make_shared<HelloworldSubscriber>();    // 스마트포인터!
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
