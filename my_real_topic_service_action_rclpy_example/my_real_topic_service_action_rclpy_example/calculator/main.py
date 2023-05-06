# Copyright 2021 OROCA
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.executors import MultiThreadedExecutor

from my_real_topic_service_action_rclpy_example.calculator.calculator import Calculator


def main(args=None):
    rclpy.init(args=args)
    try:
        calculator = Calculator()
        executor = MultiThreadedExecutor(num_threads=4) # 스레드 풀(thread pool)을 사용하여 콜백을 실행하는 것

        # 여기서 생소한 MultiThreadedExecutor이 나왔는데 MultiThreadedExecutor는 스레드 풀(thread pool)을 사용하여 콜백을 실행하는 것이다.
        # num_threads로 스레드 수를 지정 가능하며 이를 지정되지 않은 경우 multiprocessing.cpu_count()를 통해 시스템에서 가용한 스레드 수를 지정받게 된다.
        # 이 둘 모두 해당되지 않는다면 단일 스레드를 사용하게 된다.
        # 이 Executor는 콜백이 병렬로 발생하도록 허용하여 앞서 설명한 ReentrantCallbackGroup와 함께 사용하면
        # 콜백함수를 병렬로 실행할 수 있게된다.
        # 그 말인즉슨 create_subscription(), create_service(), ActionServer(), create_timer() 등의 함수를 이용하여
        # 설정한 토픽 퍼블리셔의 콜백함수, 서비스 서버의 콜백함수, 액션 서버의 콜백함수, 특정 타이머의 콜백함수 등의 병렬 처리를 설정하는 방법

        executor.add_node(calculator)
        try:
            executor.spin()
        except KeyboardInterrupt:
            calculator.get_logger().info('Keyboard Interrupt (SIGINT)')
        finally:
            executor.shutdown()
            calculator.arithmetic_action_server.destroy()
            calculator.destroy_node()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
