import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import sys
import tty
import termios
import select

class TeleopKeyboardNode(Node):
    def __init__(self):
        super().__init__('teleop_keyboard_node')
        self.left_pub = self.create_publisher(Float64, '/left_desired_velocity', 10)
        self.right_pub = self.create_publisher(Float64, '/right_desired_velocity', 10)
        self.speed = 4.7  # RPS 단위 (로봇에 맞게 조정)
        self.get_logger().info("WASD: 이동 | 스페이스: 정지 | Q: 종료")
        self.old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        self.run()

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
                elif key == ' ':
                    self.publish(0.0, 0.0)  # 정지
                elif key == 'q':
                    self.publish(0.0, 0.0)  # 종료
                    break
        except KeyboardInterrupt:
            self.publish(0.0, 0.0)
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
            self.get_logger().info("Teleop 종료")

    def publish(self, left, right):
        self.left_pub.publish(Float64(data=left))
        self.right_pub.publish(Float64(data=right))
        self.get_logger().info(f"Published - Left: {left}, Right: {right}")

    def get_key(self):
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        key = sys.stdin.read(1) if rlist else ''
        if key:
            self.get_logger().info(f"Key pressed: {key}")
        return key

def main(args=None):
    rclpy.init(args=args)
    node = TeleopKeyboardNode()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
