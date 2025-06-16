import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, String, Bool
import sys
import tty
import termios
import select

class TeleopKeyboardNode2(Node):
    def __init__(self):
        super().__init__('teleop_keyboard_node2')
        # 이동 제어 퍼블리셔
        self.left_pub = self.create_publisher(Float64, '/left_desired_velocity', 10)
        self.right_pub = self.create_publisher(Float64, '/right_desired_velocity', 10)
        # 리프트 제어 퍼블리셔
        self.lift_cmd_pub = self.create_publisher(String, '/scissor_lift_cmd', 10)
        # 펌프 완료 신호 구독자
        self.pump_done_sub = self.create_subscription(
            Bool, 'pump_done', self.pump_done_callback, 10)

        self.speed = 4.7  # RPS 단위 (로봇에 맞게 조정)
        self.get_logger().info(
            "WASD: 이동 | U: 리프트 올리기 | I: 리프트 내리기 | X: 리프트 정지 | "
            "스페이스: 이동 정지 | Q: 종료"
        )

        # 터미널 설정
        self.old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

        self.run()

    def pump_done_callback(self, msg):
        if msg.data:
            self.get_logger().info("펌프 작업 완료 신호 수신됨")

    def run(self):
        try:
            while True:
                key = self.get_key()
                if key == 'w':
                    self.publish(-self.speed, self.speed)  # 전진
                elif key == 's':
                    self.publish(self.speed, -self.speed)  # 후진
                elif key == 'a':
                    self.publish(self.speed, self.speed)  # 좌회전
                elif key == 'd':
                    self.publish(-self.speed, -self.speed)  # 우회전
                elif key == 'u':
                    self.publish_lift_cmd("up")  # 리프트 올리기
                elif key == 'i':
                    self.publish_lift_cmd("down")  # 리프트 내리기
                elif key == 'x':
                    self.publish_lift_cmd("stop")  # 리프트 정지
                elif key == ' ':
                    self.publish(0.0, 0.0)  # 이동 정지
                elif key == 'q':
                    self.publish(0.0, 0.0)  # 이동 정지
                    self.publish_lift_cmd("stop")  # 리프트 정지
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
    node = TeleopKeyboardNode2()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()