import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, String, Bool
import sys
import tty
import termios
import select

class TeleopKeyboardNode4(Node):
    def __init__(self):
        super().__init__('teleop_keyboard_node4')
        # 퍼블리셔
        self.left_pub = self.create_publisher(Float64, '/left_desired_velocity', 10)
        self.right_pub = self.create_publisher(Float64, '/right_desired_velocity', 10)
        self.lift_cmd_pub = self.create_publisher(String, '/scissor_lift_cmd', 10)
        self.start_parking_pub = self.create_publisher(Bool, '/start', 10)
        self.dismiss_pub = self.create_publisher(Bool, '/dismiss', 10)

        # 구독자
        self.pump_done_sub = self.create_subscription(Bool, '/pump_done', self.pump_done_callback, 10)
        self.parking_possible_sub = self.create_subscription(Bool, '/show_start_ui', self.parking_possible_callback, 10)
        self.align_done_sub = self.create_subscription(Bool, '/align_done', self.align_done_callback, 10)

        # 상태
        self.speed = 4.7
        self.autonomous_mode = False
        self.parking_possible = False
        self.was_parking_possible = False  # 로그 스팸 방지 플래그

        # 터미널 설정
        self.old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

        # 타이머
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info(
            "WASD: 이동 | U: 리프트 올리기 | I: 리프트 내리기 | X: 리프트 정지 | "
            "스페이스: 이동 정지 | P: 자동 주차 | M: 주차 중단 | Q: 종료"
        )

    def parking_possible_callback(self, msg):
        self.parking_possible = msg.data
        if msg.data and not self.autonomous_mode and not self.was_parking_possible:
            self.get_logger().info("자동 주차를 하시겠습니까? 'p'를 눌러 시작하세요.")
            self.was_parking_possible = True
        elif not msg.data and self.was_parking_possible:
            self.get_logger().info("주차 불가능: 마커가 시야에 없거나 조건 미충족")
            self.was_parking_possible = False

    def align_done_callback(self, msg):
        if self.autonomous_mode:
            self.autonomous_mode = False
            if msg.data:
                self.get_logger().info("자동 주차 완료. 수동 제어로 복귀합니다.")
            else:
                self.get_logger().info("자동 주차 중단됨: 마커가 시야에서 사라졌습니다. 수동 제어로 복귀합니다.")

    def pump_done_callback(self, msg):
        if msg.data:
            self.get_logger().info("펌프 작업 완료 신호 수신됨")

    def timer_callback(self):
        key = self.get_key()
        if key:
            if key == 'q':
                self.publish(0.0, 0.0)
                self.publish_lift_cmd("stop")
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
                self.get_logger().info("Teleop 종료")
                rclpy.shutdown()
            elif key == 'm' and self.autonomous_mode:
                self.dismiss_pub.publish(Bool(data=True))
                self.autonomous_mode = False
                self.get_logger().info("자동 주차 중단됨")
            elif not self.autonomous_mode:
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
                elif key == 'p' and self.parking_possible:
                    self.autonomous_mode = True
                    self.start_parking_pub.publish(Bool(data=True))
                    self.publish(0.0, 0.0)
                    self.get_logger().info("자동주차 시작됨")

    def publish(self, left, right):
        if not self.autonomous_mode:
            try:
                self.left_pub.publish(Float64(data=left))
                self.right_pub.publish(Float64(data=right))
                self.get_logger().info(f"Published - Left: {left}, Right: {right}")
            except Exception as e:
                self.get_logger().warn(f"Publish failed: {str(e)}")

    def publish_lift_cmd(self, cmd):
        try:
            self.lift_cmd_pub.publish(String(data=cmd))
            self.get_logger().info(f"Sent lift command: {cmd}")
        except Exception as e:
            self.get_logger().warn(f"Lift command publish failed: {str(e)}")

    def get_key(self):
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        return sys.stdin.read(1) if rlist else ''

    def destroy_node(self):
        # 퍼블리시 시도 전 컨텍스트 확인
        if rclpy.ok():
            self.publish(0.0, 0.0)
            self.publish_lift_cmd("stop")
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = TeleopKeyboardNode4()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("컨트롤+C로 종료. 터미널 설정 복원 중...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
    
    
# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import Float64, String, Bool
# import sys
# import tty
# import termios
# import select

# class TeleopKeyboardNode4(Node):
#     def __init__(self):
#         super().__init__('teleop_keyboard_node4')
#         # 퍼블리셔
#         self.left_pub = self.create_publisher(Float64, '/left_desired_velocity', 10)
#         self.right_pub = self.create_publisher(Float64, '/right_desired_velocity', 10)
#         self.lift_cmd_pub = self.create_publisher(String, '/scissor_lift_cmd', 10)
#         self.start_parking_pub = self.create_publisher(Bool, '/start', 10)
#         self.dismiss_pub = self.create_publisher(Bool, '/dismiss', 10)

#         # 구독자
#         self.pump_done_sub = self.create_subscription(Bool, '/pump_done', self.pump_done_callback, 10)
#         self.parking_possible_sub = self.create_subscription(Bool, '/show_start_ui', self.parking_possible_callback, 10)
#         self.align_done_sub = self.create_subscription(Bool, '/align_done', self.align_done_callback, 10)

#         # 상태
#         self.speed = 4.7
#         self.autonomous_mode = False
#         self.parking_possible = False

#         # 터미널 설정
#         self.old_settings = termios.tcgetattr(sys.stdin)
#         tty.setcbreak(sys.stdin.fileno())

#         # 타이머
#         self.timer = self.create_timer(0.1, self.timer_callback)
#         self.get_logger().info(
#             "WASD: 이동 | U: 리프트 올리기 | I: 리프트 내리기 | X: 리프트 정지 | "
#             "스페이스: 이동 정지 | P: 자동 주차 | M: 주차 중단 | Q: 종료"
#         )

#     def parking_possible_callback(self, msg):
#         self.parking_possible = msg.data
#         if msg.data and not self.autonomous_mode:
#             self.get_logger().info("자동 주차를 하시겠습니까? 'p'를 눌러 시작하세요.")

#     def align_done_callback(self, msg):
#         if msg.data:
#             self.autonomous_mode = False
#             self.get_logger().info("자동 주차 완료. 수동 제어로 복귀합니다.")

#     def pump_done_callback(self, msg):
#         if msg.data:
#             self.get_logger().info("펌프 작업 완료 신호 수신됨")

#     def timer_callback(self):
#         key = self.get_key()
#         if key:
#             if key == 'q':
#                 self.publish(0.0, 0.0)
#                 self.publish_lift_cmd("stop")
#                 termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
#                 self.get_logger().info("Teleop 종료")
#                 rclpy.shutdown()
#             elif key == 'm' and self.autonomous_mode:
#                 self.dismiss_pub.publish(Bool(data=True))
#                 self.autonomous_mode = False
#                 self.get_logger().info("자동 주차 중단됨")
#             elif not self.autonomous_mode:
#                 if key == 'w':
#                     self.publish(-self.speed, self.speed)
#                 elif key == 's':
#                     self.publish(self.speed, -self.speed)
#                 elif key == 'a':
#                     self.publish(self.speed, self.speed)
#                 elif key == 'd':
#                     self.publish(-self.speed, -self.speed)
#                 elif key == 'u':
#                     self.publish_lift_cmd("up")
#                 elif key == 'i':
#                     self.publish_lift_cmd("down")
#                 elif key == 'x':
#                     self.publish_lift_cmd("stop")
#                 elif key == ' ':
#                     self.publish(0.0, 0.0)
#                 elif key == 'p' and self.parking_possible:
#                     self.autonomous_mode = True
#                     self.start_parking_pub.publish(Bool(data=True))
#                     self.publish(0.0, 0.0)
#                     self.get_logger().info("자동주차 시작됨")

#     def publish(self, left, right):
#         if not self.autonomous_mode:
#             self.left_pub.publish(Float64(data=left))
#             self.right_pub.publish(Float64(data=right))
#             self.get_logger().info(f"Published - Left: {left}, Right: {right}")

#     def publish_lift_cmd(self, cmd):
#         self.lift_cmd_pub.publish(String(data=cmd))
#         self.get_logger().info(f"Sent lift command: {cmd}")

#     def get_key(self):
#         rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
#         return sys.stdin.read(1) if rlist else ''

#     def destroy_node(self):
#         self.publish(0.0, 0.0)
#         self.publish_lift_cmd("stop")
#         termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
#         super().destroy_node()

# def main(args=None):
#     rclpy.init(args=args)
#     node = TeleopKeyboardNode4()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()