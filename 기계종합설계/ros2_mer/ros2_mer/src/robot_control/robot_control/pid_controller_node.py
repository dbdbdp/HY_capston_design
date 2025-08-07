import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float64

class PID:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0.0
        self.prev_error = 0.0

    def compute(self, error, dt):
        if dt <= 0:
            return 0.0
        self.integral += error * dt
        self.integral = max(min(self.integral, 50.0), -50.0)
        derivative = (error - self.prev_error) / dt
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        return output

class PIDControllerNode(Node):
    def __init__(self):
        super().__init__('pid_controller_node')

        # 파라미터
        self.declare_parameter('counts_per_rev', 348.0)
        self.declare_parameter('control_frequency', 50.0)
        self.declare_parameter('pid_kp', 15.0)
        self.declare_parameter('pid_ki', 0.0)  # 치우침 방지
        self.declare_parameter('pid_kd', 0.4)
        self.declare_parameter('calibration_factor', 0.83)      #0.75 original

        self.counts_per_rev = self.get_parameter('counts_per_rev').value
        control_frequency = self.get_parameter('control_frequency').value
        kp = self.get_parameter('pid_kp').value
        ki = self.get_parameter('pid_ki').value
        kd = self.get_parameter('pid_kd').value
        self.calibration_factor = self.get_parameter('calibration_factor').value

        # PID
        self.left_pid = PID(kp, ki, kd)
        self.right_pid = PID(kp, ki, kd)
        self.straight_pid = PID(kp=1.0, ki=0.0, kd=0.05)  # 직진 보정

        # 상태
        self.left_desired_velocity = 0.0
        self.right_desired_velocity = 0.0
        self.left_counts = 0
        self.right_counts = 0
        self.prev_left_counts = 0
        self.prev_right_counts = 0
        self.last_time = None

        # 구독자
        self.left_desired_sub = self.create_subscription(
            Float64, '/left_desired_velocity', self.left_desired_callback, 10)
        self.right_desired_sub = self.create_subscription(
            Float64, '/right_desired_velocity', self.right_desired_callback, 10)
        self.left_encoder_sub = self.create_subscription(
            Int32, '/left_encoder', self.left_encoder_callback, 10)
        self.right_encoder_sub = self.create_subscription(
            Int32, '/right_encoder', self.right_encoder_callback, 10)

        # 발행자
        self.left_pwm_pub = self.create_publisher(Int32, '/left_pwm', 10)
        self.right_pwm_pub = self.create_publisher(Int32, '/right_pwm', 10)

        # 타이머
        period = 1.0 / control_frequency
        self.timer = self.create_timer(period, self.control_loop)

    def left_desired_callback(self, msg):
        self.left_desired_velocity = msg.data

    def right_desired_callback(self, msg):
        self.right_desired_velocity = msg.data

    def left_encoder_callback(self, msg):
        self.left_counts = msg.data

    def right_encoder_callback(self, msg):
        self.right_counts = msg.data

    def control_loop(self):
        current_time = self.get_clock().now()
        if self.last_time is None:
            self.last_time = current_time
            return

        dt = (current_time - self.last_time).nanoseconds / 1e9
        if dt <= 0:
            return

        # 속도 계산
        delta_left_counts = self.left_counts - self.prev_left_counts
        delta_right_counts = self.right_counts - self.prev_right_counts
        velocity_left = delta_left_counts / self.counts_per_rev / dt
        velocity_right = delta_right_counts / self.counts_per_rev / dt

        # 오차
        error_left = self.left_desired_velocity - velocity_left
        error_right = self.right_desired_velocity - velocity_right

        # PID
        pwm_left = self.left_pid.compute(error_left, dt)
        pwm_right = self.right_pid.compute(error_right, dt)

        # 캘리브레이션
        pwm_right = pwm_right * self.calibration_factor

        # 직진 보정
        error_straight = velocity_left - velocity_right
        straight_correction = self.straight_pid.compute(error_straight, dt)
        pwm_left -= straight_correction
        pwm_right += straight_correction

        # PWM 제한
        pwm_left = max(min(pwm_left, 255.0), -255.0)
        pwm_right = max(min(pwm_right, 255.0), -255.0)

        # 발행
        self.left_pwm_pub.publish(Int32(data=int(pwm_left)))
        self.right_pwm_pub.publish(Int32(data=int(pwm_right)))

        # 업데이트
        self.prev_left_counts = self.left_counts
        self.prev_right_counts = self.right_counts
        self.last_time = current_time

def main(args=None):
    rclpy.init(args=args)
    node = PIDControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

