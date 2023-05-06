import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String


class HelloworldPublisher(Node):  # Node 상속

    def __init__(self):
        super().__init__('helloworld_publisher')  # super이면 node.. 자기 부모의 인스턴스의 생성자를 초기화 해주기
        qos_profile = QoSProfile(depth=10)  # 통신 상태가 원할하지 못한 상황 퍼블리시할 데이터를 버퍼에 10개까지 저장
        self.helloworld_publisher = self.create_publisher(String, 'helloworld', qos_profile)
        # 1초마다 지정한 콜백함수를 실행하라는 것으로 아래 코드와 같이 설정하면 1초마다 publish_helloworld_msg 함수를 실행
        self.timer = self.create_timer(1, self.publish_helloworld_msg)
        self.count = 0

    def publish_helloworld_msg(self):
        msg = String()
        msg.data = 'Hello World: {0}'.format(self.count)
        self.helloworld_publisher.publish(msg)
        self.get_logger().info('Published message: {0}'.format(msg.data))
        self.count += 1


def main(args=None):
    rclpy.init(args=args)
    node = HelloworldPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
