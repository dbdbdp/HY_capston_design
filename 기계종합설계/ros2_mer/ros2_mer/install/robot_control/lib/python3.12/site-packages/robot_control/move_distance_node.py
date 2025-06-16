import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32, String
import math
import time

class MoveDistanceNode(Node):
    def __init__(self):
        super().__init__('move_distance_node')
        # 구독자 설정
        self.distance_sub = self.create_subscription(Float32, '/move_distance', self.distance_callback, 10)
        self.left_encoder_sub = self.create_subscription(Int32, '/left_encoder', self.left_encoder_callback, 10)
        self.right_encoder_sub = self.create_subscription(Int32, '/right_encoder', self.right_encoder_callback, 10)
        # 발행자 설정
        self.left_target_vel_pub = self.create_publisher(Int32, '/left_target_vel', 10)
        self.right_target_vel_pub = self.create_publisher(Int32, '/right_target_vel', 10)
        self.lift_cmd_pub = self.create_publisher(String, '/scissor_lift_cmd', 10)
        # 파라미터 초기화
        self.target_distance = None  # 목표 거리 (미터)
        self.current_distance = 0.0  # 현재 이동 거리 (미터)
        self.left_encoder_prev = 0   # 이전 왼쪽 엔코더 값
        self.right_encoder_prev = 0  # 이전 오른쪽 엔코더 값
        self.wheel_circumference = math.pi * 0.066  # 바퀴 둘레 (0.066π ≈ 0.207 미터)
        self.counts_per_revolution = 348  # 엔코더 해상도 (6 펄스 × 4 × 29 감속비)
        self.distance_threshold = 0.02    # 거리 오차 허용 범위 (5cm)
        self.target_velocity = 51         # 목표 속도 (카운트/초, 느리게 설정)
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10Hz로 주기적 실행

        self.get_logger().info("MoveDistanceNode started.")

    def distance_callback(self, msg):
        """아루코 마커에서 받은 거리를 목표 거리로 설정"""
        if self.target_distance is None:
            self.target_distance = msg.data
            self.current_distance = 0.0  # 새 목표 설정 시 현재 거리 초기화
            self.get_logger().info(f"목표 거리 설정: {self.target_distance:.3f} m")

    def left_encoder_callback(self, msg):
        """왼쪽 엔코더 값으로 이동 거리 계산"""
        delta_left = msg.data - self.left_encoder_prev
        self.left_encoder_prev = msg.data
        self.update_distance(delta_left)

    def right_encoder_callback(self, msg):
        """오른쪽 엔코더 값으로 이동 거리 계산"""
        delta_right = msg.data - self.right_encoder_prev
        self.right_encoder_prev = msg.data
        self.update_distance(delta_right)

    def update_distance(self, delta):
        """엔코더 카운트를 거리로 변환해 현재 거리 업데이트"""
        distance_per_count = self.wheel_circumference / self.counts_per_revolution
        distance_moved = delta * distance_per_count / 2  # 좌우 평균으로 계산
        self.current_distance += abs(distance_moved)  # 직선 이동이므로 절댓값 사용
        self.get_logger().debug(f"이동 거리 업데이트: {self.current_distance:.3f} m")

    def timer_callback(self):
        """주기적으로 거리 확인 및 로봇 제어"""
        if self.target_distance is None:
            return

        if self.current_distance < self.target_distance - self.distance_threshold:
            # 목표 거리까지 이동 중
            left_vel = Int32(data=self.target_velocity)
            right_vel = Int32(data=self.target_velocity)
            self.left_target_vel_pub.publish(left_vel)
            self.right_target_vel_pub.publish(right_vel)
            self.get_logger().debug(f"이동 중: 현재 {self.current_distance:.3f} m / 목표 {self.target_distance:.3f} m")
        else:
            # 목표 거리 도달, 정지 및 리프트 명령
            self.stop_robot()
            time.sleep(0.5)
            self.get_logger().info(f"목표 거리 도달: {self.current_distance:.3f} m, 로봇 정지")
            lift_cmd_msg = String()
            lift_cmd_msg.data = "up"
            self.lift_cmd_pub.publish(lift_cmd_msg)
            self.get_logger().info("리프트 'up' 명령 발행")
            # 초기화 (다음 목표 대기)
            self.target_distance = None
            self.current_distance = 0.0

    def stop_robot(self):
        """로봇 정지 명령"""
        left_vel = Int32(data=0)
        right_vel = Int32(data=0)
        self.left_target_vel_pub.publish(left_vel)
        self.right_target_vel_pub.publish(right_vel)

def main(args=None):
    rclpy.init(args=args)
    node = MoveDistanceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()