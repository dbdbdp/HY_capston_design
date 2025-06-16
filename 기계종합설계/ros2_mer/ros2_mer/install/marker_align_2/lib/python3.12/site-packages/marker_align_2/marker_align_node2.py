import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float64, Bool, String
import time

class MarkerAlign(Node):
    def __init__(self):
        super().__init__('MarkerAlign')

        # 설정값
        self.center_x = 350
        self.deadband = 30
        self.fixed_velocity = 4.0
        self.distance_threshold = 0.28
        self.extra_forward_time = 0.55
        self.pause_duration = 1.0
        self.centering_pause_duration = 0.5
        self.ui_trigger_pitch_deg = 20.0
        self.sequence_timeout = 40.0
        self.marker_timeout = 1.0  # 마커 데이터 최신성 타임아웃 (초)

        # 상태 변수
        self.mode = 'WAIT'
        self.active = False
        self.cx_error = None
        self.pitch = None
        self.pitch_deg = None
        self.distance = None
        self.done_signal_sent = False
        self.lift_command_sent = False
        self.pause_start_time = None
        self.extra_start_time = None
        self.centering_step_time = None
        self.ui_shown = False
        self.sequence_start_time = None
        self.last_marker_time = None  # 마지막 마커 데이터 시간

        # 구독자
        self.cx_sub = self.create_subscription(Float32, '/marker_cx', self.marker_callback, 10)
        self.pitch_sub = self.create_subscription(Float32, '/marker_pitch', self.pitch_callback, 10)
        self.dist_sub = self.create_subscription(Float32, '/move_distance', self.distance_callback, 10)
        self.switch_sta = self.create_subscription(Bool, '/start', self.start_callback, 10)
        self.switch_dis = self.create_subscription(Bool, '/dismiss', self.dismiss_callback, 10)

        # 발행자
        self.left_target_pub = self.create_publisher(Float64, '/left_desired_velocity', 10)
        self.right_target_pub = self.create_publisher(Float64, '/right_desired_velocity', 10)
        self.align_done_pub = self.create_publisher(Bool, '/align_done', 10)
        self.ui_pub = self.create_publisher(Bool, '/show_start_ui', 10)
        self.lift_cmd_pub = self.create_publisher(String, '/scissor_lift_cmd', 10)

        # 메인 루프 타이머
        self.timer = self.create_timer(0.1, self.main_loop)
        self.get_logger().info("마커 정렬 및 접근 노드가 시작되었습니다.")

    def marker_callback(self, msg):
        self.cx_error = msg.data - self.center_x
        self.last_marker_time = time.time()
        if self.pitch is not None and abs(self.pitch) < self.ui_trigger_pitch_deg:
            if not self.ui_shown:
                self.ui_pub.publish(Bool(data=True))
                self.get_logger().info("[UI] pitch 정렬 가능 → start UI 표시 요청")
                self.ui_shown = True
        else:
            self.ui_shown = False
            self.ui_pub.publish(Bool(data=False))

    def pitch_callback(self, msg):
        self.pitch = msg.data
        self.pitch_deg = abs(msg.data)
        self.last_marker_time = time.time()

    def distance_callback(self, msg):
        self.distance = msg.data
        self.last_marker_time = time.time()

    def start_callback(self, msg):
        if msg.data:
            now = time.time()
            if self.last_marker_time is None or now - self.last_marker_time > self.marker_timeout:
                self.get_logger().warn("[START] 마커 데이터가 최신이 아니므로 시작 명령을 무시합니다.")
                return
            if not self.ui_shown:
                self.get_logger().warn("[START] UI가 표시되지 않았기 때문에 시작 명령을 무시합니다.")
                return
            self.active = True
            self.mode = 'CENTERING'
            self.done_signal_sent = False
            self.lift_command_sent = False
            self.sequence_start_time = now
            self.get_logger().info("[START] 정렬 시퀀스 시작")

    def dismiss_callback(self, msg):
        if msg.data:
            if not self.active:
                self.get_logger().warn("[DISMISS] 현재 실행 중인 시퀀스가 없어 명령을 무시합니다.")
                return
            self._stop_sequence(reason="DISMISS 명령 수신")

    def _stop_sequence(self, reason=""):
        self.active = False
        self.mode = 'WAIT'
        self.left_target_pub.publish(Float64(data=0.0))
        self.right_target_pub.publish(Float64(data=0.0))  # right 에러 수정
        self.get_logger().info(f"[STOP] 정렬 시퀀스 중단됨. 사유: {reason}")

    def main_loop(self):
        now = time.time()

        # 마커 데이터 최신성 체크
        if self.last_marker_time is not None and now - self.last_marker_time > self.marker_timeout:
            self.cx_error = None
            self.pitch = None
            self.pitch_deg = None
            self.distance = None
            if self.ui_shown:
                self.ui_shown = False
                self.ui_pub.publish(Bool(data=False))
            if self.active:
                self._stop_sequence(reason="마커 데이터 손실")
            return

        if not self.active:
            return

        # 타임아웃 체크
        if self.sequence_start_time and now - self.sequence_start_time > self.sequence_timeout:
            self._stop_sequence(reason="TIMEOUT: 시퀀스가 제한 시간 내 완료되지 않음")
            return

        if self.mode == 'CENTERING':
            if self.cx_error is None or self.pitch_deg is None:
                self.get_logger().warn("마커 중심 또는 피치 데이터 수신 대기 중...")
                return
            if abs(self.cx_error) < self.deadband and self.pitch_deg < 15.0:
                self.mode = 'FORWARD'
                self.get_logger().info("[CENTERING] 정렬 완료 → 전진 이동으로 전환")
            else:
                if self.cx_error > self.deadband or self.pitch_deg > 15.0:
                    left_velocity = -(self.fixed_velocity - 0.5)
                    right_velocity = -(self.fixed_velocity - 0.5)
                elif self.cx_error < -self.deadband:
                    left_velocity = (self.fixed_velocity - 0.5)
                    right_velocity = (self.fixed_velocity - 0.5)
                else:
                    left_velocity = 0.0
                    right_velocity = 0.0
                self.left_target_pub.publish(Float64(data=left_velocity))
                self.right_target_pub.publish(Float64(data=right_velocity))
                self.get_logger().info(f"[CENTERING] 회전 수행 → L:{left_velocity:.2f}, R:{right_velocity:.2f}")
                self.mode = 'CENTERING_PAUSE'
                self.centering_step_time = now

        elif self.mode == 'CENTERING_PAUSE':
            if now - self.centering_step_time < self.centering_pause_duration:
                self.left_target_pub.publish(Float64(data=0.0))
                self.right_target_pub.publish(Float64(data=0.0))
                self.get_logger().info(f"[CENTERING_PAUSE] 판단 대기 중... ({now - self.centering_step_time:.1f}s)")
            else:
                self.mode = 'CENTERING'

        elif self.mode == 'FORWARD':
            if self.distance is not None and self.distance < self.distance_threshold:
                self.mode = 'PAUSE'
                self.pause_start_time = now
                self.get_logger().info("[FORWARD] 목표 거리 도달 → 일시 정지 단계로 전환")
            else:
                self.left_target_pub.publish(Float64(data=-self.fixed_velocity))
                self.right_target_pub.publish(Float64(data=self.fixed_velocity))
                self.get_logger().info(f"[FORWARD] 거리: {self.distance:.2f} → 전진 중")

        elif self.mode == 'PAUSE':
            if now - self.pause_start_time < self.pause_duration:
                self.left_target_pub.publish(Float64(data=0.0))
                self.right_target_pub.publish(Float64(data=0.0))
                self.get_logger().info(f"[PAUSE] 일시 정지 중... ({now - self.pause_start_time:.1f}s)")
            else:
                self.mode = 'EXTRA_FORWARD'
                self.extra_start_time = now
                self.get_logger().info("[PAUSE] 일시 정지 완료 → 추가 전진 단계로 전환")

        elif self.mode == 'EXTRA_FORWARD':
            if now - self.extra_start_time < self.extra_forward_time:
                self.left_target_pub.publish(Float64(data=-self.fixed_velocity))
                self.right_target_pub.publish(Float64(data=self.fixed_velocity))
                self.get_logger().info(f"[EXTRA] 추가 전진 중... ({now - self.extra_start_time:.1f}s)")
            else:
                self.left_target_pub.publish(Float64(data=0.0))
                self.right_target_pub.publish(Float64(data=0.0))
                self.mode = 'DONE'
                self.get_logger().info("[DONE] 전진 완료 → 정지")

        elif self.mode == 'DONE':
            if not self.done_signal_sent:
                self.align_done_pub.publish(Bool(data=True))
                self.get_logger().info("[DONE] 정렬 완료 신호 발행")
                self.done_signal_sent = True
            if not self.lift_command_sent:
                self.lift_cmd_pub.publish(String(data="up"))
                self.get_logger().info("[DONE] 리프트 올리기 명령 발행")
                self.lift_command_sent = True

def main(args=None):
    rclpy.init(args=args)
    node = MarkerAlign()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("컨트롤+C로 종료.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import Float32, Float64, Bool, String
# import time

# class MarkerAlign(Node):
#     def __init__(self):
#         super().__init__('MarkerAlign')

#         # 설정값 (실험 성공 값 유지)
#         self.center_x = 350
#         self.deadband = 30
#         self.fixed_velocity = 4.0
#         self.distance_threshold = 0.28
#         self.extra_forward_time = 0.55
#         self.pause_duration = 1.0
#         self.centering_pause_duration = 0.5
#         self.ui_trigger_pitch_deg = 20.0            # original value : 30
#         self.sequence_timeout = 40.0

#         # 상태 변수
#         self.mode = 'WAIT'
#         self.active = False
#         self.cx_error = None
#         self.pitch = None
#         self.pitch_deg = None
#         self.distance = None
#         self.done_signal_sent = False
#         self.lift_command_sent = False
#         self.pause_start_time = None
#         self.extra_start_time = None
#         self.centering_step_time = None
#         self.ui_shown = False
#         self.sequence_start_time = None

#         # 구독자
#         self.cx_sub = self.create_subscription(Float32, '/marker_cx', self.marker_callback, 10)
#         self.pitch_sub = self.create_subscription(Float32, '/marker_pitch', self.pitch_callback, 10)
#         self.dist_sub = self.create_subscription(Float32, '/move_distance', self.distance_callback, 10)
#         self.switch_sta = self.create_subscription(Bool, '/start', self.start_callback, 10)
#         self.switch_dis = self.create_subscription(Bool, '/dismiss', self.dismiss_callback, 10)

#         # 발행자
#         self.left_target_pub = self.create_publisher(Float64, '/left_desired_velocity', 10)
#         self.right_target_pub = self.create_publisher(Float64, '/right_desired_velocity', 10)
#         self.align_done_pub = self.create_publisher(Bool, '/align_done', 10)
#         self.ui_pub = self.create_publisher(Bool, '/show_start_ui', 10)
#         self.lift_cmd_pub = self.create_publisher(String, '/scissor_lift_cmd', 10)

#         # 루프 시작
#         self.timer = self.create_timer(0.1, self.main_loop)
#         self.get_logger().info("마커 정렬 및 접근 노드가 시작되었습니다.")

#     def marker_callback(self, msg):
#         self.cx_error = msg.data - self.center_x

#         if self.pitch is not None and abs(self.pitch) < self.ui_trigger_pitch_deg:
#             if not self.ui_shown:
#                 self.ui_pub.publish(Bool(data=True))
#                 self.get_logger().info("[UI] pitch 정렬 가능 → start UI 표시 요청")
#                 self.ui_shown = True
#         else:
#             self.ui_shown = False

#     def pitch_callback(self, msg):
#         self.pitch = msg.data
#         self.pitch_deg = abs(msg.data)

#     def distance_callback(self, msg):
#         self.distance = msg.data

#     def start_callback(self, msg):
#         if msg.data:
#             if not self.ui_shown:
#                 self.get_logger().warn("[START] UI가 표시되지 않았기 때문에 시작 명령을 무시합니다.")
#                 return
#             self.active = True
#             self.mode = 'CENTERING'
#             self.done_signal_sent = False
#             self.lift_command_sent = False
#             self.sequence_start_time = time.time()
#             self.get_logger().info("[START] 정렬 시퀀스 시작")

#     def dismiss_callback(self, msg):
#         if msg.data:
#             if not self.active:
#                 self.get_logger().warn("[DISMISS] 현재 실행 중인 시퀀스가 없어 명령을 무시합니다.")
#                 return
#             self._stop_sequence(reason="DISMISS 명령 수신")

#     def _stop_sequence(self, reason=""):
#         self.active = False
#         self.mode = 'WAIT'
#         self.left_target_pub.publish(Float64(data=0.0))
#         self.right_target_pub.publish(Float64(data=0.0))
#         self.get_logger().info(f"[STOP] 정렬 시퀀스 중단됨. 사유: {reason}")

#     def main_loop(self):
#         now = time.time()

#         if not self.active:
#             return

#         # Timeout 검사
#         if self.sequence_start_time and now - self.sequence_start_time > self.sequence_timeout:
#             self._stop_sequence(reason="TIMEOUT: 시퀀스가 제한 시간 내 완료되지 않음")
#             return

#         if self.mode == 'CENTERING':
#             if self.cx_error is None or self.pitch_deg is None:
#                 self.get_logger().warn("마커 중심 또는 피치 데이터 수신 대기 중...")
#                 return

#             if abs(self.cx_error) < self.deadband and self.pitch_deg < 15.0:    #original value: 10
#                 self.mode = 'FORWARD'
#                 self.get_logger().info("[CENTERING] 정렬 완료 → 전진 이동으로 전환")
#             else:
#                 if self.cx_error > self.deadband or self.pitch_deg > 15.0:      #original value: 10
#                     left_velocity = -(self.fixed_velocity - 0.5)
#                     right_velocity = -(self.fixed_velocity - 0.5)
#                 elif self.cx_error < -self.deadband:
#                     left_velocity = (self.fixed_velocity - 0.5)
#                     right_velocity = (self.fixed_velocity - 0.5)
#                 else:
#                     left_velocity = 0.0
#                     right_velocity = 0.0

#                 self.left_target_pub.publish(Float64(data=left_velocity))
#                 self.right_target_pub.publish(Float64(data=right_velocity))
#                 self.get_logger().info(f"[CENTERING] 회전 수행 → L:{left_velocity:.2f}, R:{right_velocity:.2f}")

#                 self.mode = 'CENTERING_PAUSE'
#                 self.centering_step_time = now

#         elif self.mode == 'CENTERING_PAUSE':
#             if now - self.centering_step_time < self.centering_pause_duration:
#                 self.left_target_pub.publish(Float64(data=0.0))
#                 self.right_target_pub.publish(Float64(data=0.0))
#                 self.get_logger().info(f"[CENTERING_PAUSE] 판단 대기 중... ({now - self.centering_step_time:.1f}s)")
#             else:
#                 self.mode = 'CENTERING'

#         elif self.mode == 'FORWARD':
#             if self.distance is not None and self.distance < self.distance_threshold:
#                 self.mode = 'PAUSE'
#                 self.pause_start_time = now
#                 self.get_logger().info("[FORWARD] 목표 거리 도달 → 일시 정지 단계로 전환")
#             else:
#                 self.left_target_pub.publish(Float64(data=-self.fixed_velocity))
#                 self.right_target_pub.publish(Float64(data=self.fixed_velocity))
#                 self.get_logger().info(f"[FORWARD] 거리: {self.distance:.2f} → 전진 중")

#         elif self.mode == 'PAUSE':
#             if now - self.pause_start_time < self.pause_duration:
#                 self.left_target_pub.publish(Float64(data=0.0))
#                 self.right_target_pub.publish(Float64(data=0.0))
#                 self.get_logger().info(f"[PAUSE] 일시 정지 중... ({now - self.pause_start_time:.1f}s)")
#             else:
#                 self.mode = 'EXTRA_FORWARD'
#                 self.extra_start_time = now
#                 self.get_logger().info("[PAUSE] 일시 정지 완료 → 추가 전진 단계로 전환")

#         elif self.mode == 'EXTRA_FORWARD':
#             if now - self.extra_start_time < self.extra_forward_time:
#                 self.left_target_pub.publish(Float64(data=-self.fixed_velocity))
#                 self.right_target_pub.publish(Float64(data=self.fixed_velocity))
#                 self.get_logger().info(f"[EXTRA] 추가 전진 중... ({now - self.extra_start_time:.1f}s)")
#             else:
#                 self.left_target_pub.publish(Float64(data=0.0))
#                 self.right_target_pub.publish(Float64(data=0.0))
#                 self.mode = 'DONE'
#                 self.get_logger().info("[DONE] 전진 완료 → 정지")

#         elif self.mode == 'DONE':
#             if not self.done_signal_sent:
#                 self.align_done_pub.publish(Bool(data=True))
#                 self.get_logger().info("[DONE] 정렬 완료 신호 발행")
#                 self.done_signal_sent = True
#             if not self.lift_command_sent:
#                 self.lift_cmd_pub.publish(String(data="up"))
#                 self.get_logger().info("[DONE] 리프트 올리기 명령 발행")
#                 self.lift_command_sent = True

# def main(args=None):
#     rclpy.init(args=args)
#     node = MarkerAlign()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()


# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import Float32, Float64, Bool, String
# import time

# class MarkerAlign(Node):
#     def __init__(self):
#         super().__init__('MarkerAlign')

#         # 설정값 (실험 성공 값 유지)
#         self.center_x = 350
#         self.deadband = 30
#         self.fixed_velocity = 4.0
#         self.distance_threshold = 0.28
#         self.extra_forward_time = 0.55
#         self.pause_duration = 1.0
#         self.centering_pause_duration = 0.5
#         self.ui_trigger_pitch_deg = 20.0            # original value : 30
#         self.sequence_timeout = 40.0

#         # 상태 변수
#         self.mode = 'WAIT'
#         self.active = False
#         self.cx_error = None
#         self.pitch = None
#         self.pitch_deg = None
#         self.distance = None
#         self.done_signal_sent = False
#         self.lift_command_sent = False
#         self.pause_start_time = None
#         self.extra_start_time = None
#         self.centering_step_time = None
#         self.ui_shown = False
#         self.sequence_start_time = None

#         # 구독자
#         self.cx_sub = self.create_subscription(Float32, '/marker_cx', self.marker_callback, 10)
#         self.pitch_sub = self.create_subscription(Float32, '/marker_pitch', self.pitch_callback, 10)
#         self.dist_sub = self.create_subscription(Float32, '/move_distance', self.distance_callback, 10)
#         self.switch_sta = self.create_subscription(Bool, '/start', self.start_callback, 10)
#         self.switch_dis = self.create_subscription(Bool, '/dismiss', self.dismiss_callback, 10)

#         # 발행자
#         self.left_target_pub = self.create_publisher(Float64, '/left_desired_velocity', 10)
#         self.right_target_pub = self.create_publisher(Float64, '/right_desired_velocity', 10)
#         self.align_done_pub = self.create_publisher(Bool, '/align_done', 10)
#         self.ui_pub = self.create_publisher(Bool, '/show_start_ui', 10)
#         self.lift_cmd_pub = self.create_publisher(String, '/scissor_lift_cmd', 10)

#         # 루프 시작
#         self.timer = self.create_timer(0.1, self.main_loop)
#         self.get_logger().info("마커 정렬 및 접근 노드가 시작되었습니다.")

#     def marker_callback(self, msg):
#         self.cx_error = msg.data - self.center_x

#         if self.pitch is not None and abs(self.pitch) < self.ui_trigger_pitch_deg:
#             if not self.ui_shown:
#                 self.ui_pub.publish(Bool(data=True))
#                 self.get_logger().info("[UI] pitch 정렬 가능 → start UI 표시 요청")
#                 self.ui_shown = True
#         else:
#             self.ui_shown = False

#     def pitch_callback(self, msg):
#         self.pitch = msg.data
#         self.pitch_deg = abs(msg.data)

#     def distance_callback(self, msg):
#         self.distance = msg.data

#     def start_callback(self, msg):
#         if msg.data:
#             if not self.ui_shown:
#                 self.get_logger().warn("[START] UI가 표시되지 않았기 때문에 시작 명령을 무시합니다.")
#                 return
#             self.active = True
#             self.mode = 'CENTERING'
#             self.done_signal_sent = False
#             self.lift_command_sent = False
#             self.sequence_start_time = time.time()
#             self.get_logger().info("[START] 정렬 시퀀스 시작")

#     def dismiss_callback(self, msg):
#         if msg.data:
#             if not self.active:
#                 self.get_logger().warn("[DISMISS] 현재 실행 중인 시퀀스가 없어 명령을 무시합니다.")
#                 return
#             self._stop_sequence(reason="DISMISS 명령 수신")

#     def _stop_sequence(self, reason=""):
#         self.active = False
#         self.mode = 'WAIT'
#         self.left_target_pub.publish(Float64(data=0.0))
#         self.right_target_pub.publish(Float64(data=0.0))
#         self.get_logger().info(f"[STOP] 정렬 시퀀스 중단됨. 사유: {reason}")

#     def main_loop(self):
#         now = time.time()

#         if not self.active:
#             return

#         # Timeout 검사
#         if self.sequence_start_time and now - self.sequence_start_time > self.sequence_timeout:
#             self._stop_sequence(reason="TIMEOUT: 시퀀스가 제한 시간 내 완료되지 않음")
#             return

#         if self.mode == 'CENTERING':
#             if self.cx_error is None or self.pitch_deg is None:
#                 self.get_logger().warn("마커 중심 또는 피치 데이터 수신 대기 중...")
#                 return

#             if abs(self.cx_error) < self.deadband and self.pitch_deg < 150.0:    #original value: 10
#                 self.mode = 'FORWARD'
#                 self.get_logger().info("[CENTERING] 정렬 완료 → 전진 이동으로 전환")
#             else:
#                 if self.cx_error > self.deadband or self.pitch_deg > 15.0:      #original value: 10
#                     left_velocity = -(self.fixed_velocity - 0.5)
#                     right_velocity = -(self.fixed_velocity - 0.5)
#                 elif self.cx_error < -self.deadband:
#                     left_velocity = (self.fixed_velocity - 0.5)
#                     right_velocity = (self.fixed_velocity - 0.5)
#                 else:
#                     left_velocity = 0.0
#                     right_velocity = 0.0

#                 self.left_target_pub.publish(Float64(data=left_velocity))
#                 self.right_target_pub.publish(Float64(data=right_velocity))
#                 self.get_logger().info(f"[CENTERING] 회전 수행 → L:{left_velocity:.2f}, R:{right_velocity:.2f}")

#                 self.mode = 'CENTERING_PAUSE'
#                 self.centering_step_time = now

#         elif self.mode == 'CENTERING_PAUSE':
#             if now - self.centering_step_time < self.centering_pause_duration:
#                 self.left_target_pub.publish(Float64(data=0.0))
#                 self.right_target_pub.publish(Float64(data=0.0))
#                 self.get_logger().info(f"[CENTERING_PAUSE] 판단 대기 중... ({now - self.centering_step_time:.1f}s)")
#             else:
#                 self.mode = 'CENTERING'

#         elif self.mode == 'FORWARD':
#             if self.distance is not None and self.distance < self.distance_threshold:
#                 self.mode = 'PAUSE'
#                 self.pause_start_time = now
#                 self.get_logger().info("[FORWARD] 목표 거리 도달 → 일시 정지 단계로 전환")
#             else:
#                 self.left_target_pub.publish(Float64(data=-self.fixed_velocity))
#                 self.right_target_pub.publish(Float64(data=self.fixed_velocity))
#                 self.get_logger().info(f"[FORWARD] 거리: {self.distance:.2f} → 전진 중")

#         elif self.mode == 'PAUSE':
#             if now - self.pause_start_time < self.pause_duration:
#                 self.left_target_pub.publish(Float64(data=0.0))
#                 self.right_target_pub.publish(Float64(data=0.0))
#                 self.get_logger().info(f"[PAUSE] 일시 정지 중... ({now - self.pause_start_time:.1f}s)")
#             else:
#                 self.mode = 'EXTRA_FORWARD'
#                 self.extra_start_time = now
#                 self.get_logger().info("[PAUSE] 일시 정지 완료 → 추가 전진 단계로 전환")

#         elif self.mode == 'EXTRA_FORWARD':
#             if now - self.extra_start_time < self.extra_forward_time:
#                 self.left_target_pub.publish(Float64(data=-self.fixed_velocity))
#                 self.right_target_pub.publish(Float64(data=self.fixed_velocity))
#                 self.get_logger().info(f"[EXTRA] 추가 전진 중... ({now - self.extra_start_time:.1f}s)")
#             else:
#                 self.left_target_pub.publish(Float64(data=0.0))
#                 self.right_target_pub.publish(Float64(data=0.0))
#                 self.mode = 'DONE'
#                 self.get_logger().info("[DONE] 전진 완료 → 정지")

#         elif self.mode == 'DONE':
#             if not self.done_signal_sent:
#                 self.align_done_pub.publish(Bool(data=True))
#                 self.get_logger().info("[DONE] 정렬 완료 신호 발행")
#                 self.done_signal_sent = True
#             if not self.lift_command_sent:
#                 self.lift_cmd_pub.publish(String(data="up"))
#                 self.get_logger().info("[DONE] 리프트 올리기 명령 발행")
#                 self.lift_command_sent = True

# def main(args=None):
#     rclpy.init(args=args)
#     node = MarkerAlign()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()