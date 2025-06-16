import rclpy  # ROS 2 Python 클라이언트 라이브러리 임포트
from rclpy.node import Node  # ROS 2의 노드 기본 클래스 임포트
from gpiozero import OutputDevice  # GPIO 핀의 ON/OFF 제어를 위한 클래스
from std_msgs.msg import Bool  # ROS 2에서 사용할 메시지 타입: Bool (True/False)
import time  # 시간 지연을 위한 표준 파이썬 라이브러리

class PumpNode(Node):  # ROS 2 노드 정의. 노드 이름은 pump_node로 설정
    def __init__(self):
        super().__init__('pump_node')  # 부모 클래스 초기화 + 노드 이름 설정

        # GPIO 핀 번호 5번에 연결된 L9110 IN1 제어용 객체 생성
        self.in1 = OutputDevice(5)

        # GPIO 핀 번호 6번에 연결된 L9110 IN2 제어용 객체 생성
        self.in2 = OutputDevice(6)

        # lift_done 토픽 구독 설정 (리프트 완료 신호를 기다림)
        self.subscription = self.create_subscription(
            Bool,                # 메시지 타입: std_msgs/msg/Bool
            'lift_done',         # 토픽 이름: /lift_done
            self.pump_move,  # 메시지를 받으면 실행될 콜백 함수
            10                   # QoS (버퍼 용량)
        )

        # 구독 상태를 로그로 출력
        self.get_logger().info("pump_node: 'lift_done' 토픽 구독 중")

        # 펌프 완료 신호 송신을 위한 퍼블리셔 생성
        self.publisher = self.create_publisher(
            Bool,
            'pump_done',
            10
        )

    def pump_move(self, msg):  # lift_done 토픽 메시지를 수신했을 때 실행되는 함수
        if msg.data:  # msg.data가 True일 경우 (즉, 리프트 완료 신호가 도착했을 때)

            # 로그 출력: 리프트 완료 확인 및 대기 안내
            self.get_logger().info("lift_done 신호 수신됨. 3초 대기 후 펌프 작동")

            time.sleep(3)  # 3초간 대기 (리프트가 완전히 멈추는 시간 확보)

            # 펌프 ON: IN1 ON, IN2 OFF → L9110을 통해 펌프가 정방향으로 회전
            self.in1.on()  # IN1 핀을 HIGH로 설정
            self.in2.off()  # IN2 핀을 LOW로 설정
            self.get_logger().info("펌프 ON (2초간 물 주기)")

            time.sleep(5)  # 2초간 물을 줌

            # 펌프 OFF: IN1 OFF, IN2 OFF → 펌프 정지
            self.in1.off()  # IN1 핀 LOW
            self.in2.off()  # IN2 핀 LOW
            self.get_logger().info("펌프 OFF (물 주기 완료)")

            # 펌프 완료 신호 발행
            done_msg = Bool()
            done_msg.data = True
            self.publisher.publish(done_msg)
            self.get_logger().info("pump_done 토픽 발행 완료")


def main(args=None):
    rclpy.init(args=args)
    node = PumpNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()