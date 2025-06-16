import rclpy  # ROS 2 Python 라이브러리
from rclpy.node import Node  # ROS 2 노드 클래스
from gpiozero import OutputDevice  # GPIO 제어용
import time  # 지연을 위한 표준 라이브러리

class PumpTestNode(Node):  # 테스트용 ROS 2 노드 정의
    def __init__(self):
        super().__init__('pump_testnode')  # 노드 이름: pump_testnode

        # GPIO 17 → IN1 (L9110)
        self.in1 = OutputDevice(5)

        # GPIO 27 → IN2 (L9110)
        self.in2 = OutputDevice(6)

        self.get_logger().info("펌프 테스트 시작: 3초 대기 후 작동")

        # 테스트 루틴 실행
        self.run_test()

    def run_test(self):
        time.sleep(3)  # 리프트 대기 상황 시뮬레이션

        self.get_logger().info("펌프 ON")
        self.in1.on()       # IN1 ON
        self.in2.off()      # IN2 OFF → 펌프 정방향 작동
        time.sleep(5)       # 2초간 물 주기

        self.get_logger().info("펌프 OFF")
        self.in1.off()      # IN1 OFF
        self.in2.off()      # IN2 OFF → 정지

        self.get_logger().info("펌프 테스트 완료")

def main(args=None):
    rclpy.init(args=args)
    node = PumpTestNode()
    time.sleep(0.5)  # 로그 출력 기다림
    node.destroy_node()
    rclpy.shutdown()
