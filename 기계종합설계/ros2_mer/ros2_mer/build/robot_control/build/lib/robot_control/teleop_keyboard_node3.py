import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, String, Bool
import sys
import tty
import termios
import select
import time

class TeleopKeyboardNode3(Node):
    def __init__(self):
        super().__init__('teleop_keyboard_node3')

        # 퍼블리셔
        self.left_pub = self.create_publisher(Float64, '/left_desired_velocity', 10)
        self.right_pub = self.create_publisher(Float64, '/right_desired_velocity', 10)
        self.lift_cmd_pub = self.create_publisher(String, '/scissor_lift_cmd', 10)
        self.start_pub = self.create_publisher(Bool, '/start', 10)
        self.dismiss_pub = self.create_publisher(Bool, '/dismiss', 10)

        # 서브스크라이버
        self.pump_done_sub = self.create_subscription(Bool, '/pump_done', self.pump_done_callback, 10)
        self.ui_sub = self.create_subscription(Bool, '/show_start_ui', self.show_ui_callback, 10)

        # 속도 및 UI 상태
        self.speed = 4.7
        self.ui_hold_duration = 3.0  # UI 띄운 후 입력 잠금 시간 (초)
        self.ui_hold_start_time = None

        self.get_logger().info(
            "WASD: 이동 | U: 리프트 올리기 | I: 리프트 내리기 | X: 리프트 정지 | "
            "G: 정렬 시작(UI 출력 시) | H: 정렬 중단 | 스페이스: 이동 정지 | Q: 종료"
        )

        # 터미널 설정
        self.old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

        self.run()

    def pump_done_callback(self, msg):
        if msg.data:
            self.get_logger().info("펌프 작업 완료 신호 수신됨")

    def show_ui_callback(self, msg):
        if msg.data:
            self.publish(0.0, 0.0)
            self.ui_hold_start_time = time.time()
            self.get_logger().info("▶▶▶ pitch 기준 만족! 로봇 정지됨. 'G' 키로 정렬을 시작하세요.")
            self.get_logger().info(f"(키보드 입력은 {self.ui_hold_duration}초간 잠시 비활성화됩니다.)")

    def run(self):
        try:
            while True:
                # UI 띄운 후 일정 시간 동안 입력 차단
                if self.ui_hold_start_time is not None:
                    elapsed = time.time() - self.ui_hold_start_time
                    if elapsed < self.ui_hold_duration:
                        continue
                    else:
                        self.ui_hold_start_time = None

                key = self.get_key()
                if key == 'w':
                    self.publish(-self.speed, self.speed)
                elif key == 's':
                    self.publish(self.speed, -self.speed)
                elif key == 'a':
                    self.publish(self.speed, self.speed)
                elif key == 'd':
                    self.publish(-self.speed, -self.speed)
                elif key == 'u':
                    self.publish_lift_cmd("up")
                elif key == 'i':
                    self.publish_lift_cmd("down")
                elif key == 'x':
                    self.publish_lift_cmd("stop")
                elif key == ' ':
                    self.publish(0.0, 0.0)
                elif key == 'g':
                    self.start_pub.publish(Bool(data=True))
                    self.get_logger().info("▶ START 명령 발행 (/start)")
                elif key == 'h':
                    self.dismiss_pub.publish(Bool(data=True))
                    self.get_logger().info("■ DISMISS 명령 발행 (/dismiss)")
                elif key == 'q':
                    self.publish(0.0, 0.0)
                    self.publish_lift_cmd("stop")
                    break
        except KeyboardInterrupt:
            self.publish(0.0, 0.0)
            self.publish_lift_cmd("stop")
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
            self.publish(0.0, 0.0)
            self.publish_lift_cmd("stop")
            self.get_logger().info("Teleop 종료")

    def publish(self, left, right):
        self.left_pub.publish(Float64(data=left))
        self.right_pub.publish(Float64(data=right))
        self.get_logger().info(f"Published - Left: {left}, Right: {right}")

    def publish_lift_cmd(self, cmd):
        msg = String()
        msg.data = cmd
        self.lift_cmd_pub.publish(msg)
        self.get_logger().info(f"Sent lift command: {cmd}")

    def get_key(self):
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        return sys.stdin.read(1) if rlist else ''

    def destroy_node(self):
        self.publish(0.0, 0.0)
        self.publish_lift_cmd("stop")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = TeleopKeyboardNode3()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
