import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from adafruit_pca9685 import PCA9685
import board
import busio
import threading
import time

# PWM 채널 설정
LEFT_LPWM_CHANNEL = 0
LEFT_RPWM_CHANNEL = 1
RIGHT_LPWM_CHANNEL = 4
RIGHT_RPWM_CHANNEL = 5

# EN 핀
L_LEN = 2
L_REN = 3
R_LEN = 6
R_REN = 7

PWM_FREQ = 1000
MAX_DUTY = 0xFFFF

SOFT_START_THRESHOLD = 30    # 이전값과 30 이상 차이나면 soft start 적용
SOFT_START_DURATION = 0.2    # soft start 전체 시간 (초)
SOFT_START_STEPS = 10        # 보간 단계 수

class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller_node')

        # I2C 및 PCA9685 초기화
        i2c = busio.I2C(board.SCL, board.SDA)
        self.pca = PCA9685(i2c)
        self.pca.frequency = PWM_FREQ

        # EN 핀 항상 HIGH
        for ch in [L_LEN, L_REN, R_LEN, R_REN]:
            self.pca.channels[ch].duty_cycle = MAX_DUTY

        # 현재 PWM 상태 저장
        self.prev_left_pwm = 0
        self.prev_right_pwm = 0

        # subscriber
        self.left_sub = self.create_subscription(Int32, '/left_pwm', self.left_callback, 10)
        self.right_sub = self.create_subscription(Int32, '/right_pwm', self.right_callback, 10)

        self.get_logger().info("MotorController node started.")

    def left_callback(self, msg):
        target_pwm = msg.data
        if abs(target_pwm - self.prev_left_pwm) >= SOFT_START_THRESHOLD:
            threading.Thread(target=self.soft_start_pwm,
                             args=(target_pwm, self.prev_left_pwm, 'left'), daemon=True).start()
        else:
            self.set_left_pwm(target_pwm)
        self.prev_left_pwm = target_pwm

    def right_callback(self, msg):
        target_pwm = msg.data
        if abs(target_pwm - self.prev_right_pwm) >= SOFT_START_THRESHOLD:
            threading.Thread(target=self.soft_start_pwm,
                             args=(target_pwm, self.prev_right_pwm, 'right'), daemon=True).start()
        else:
            self.set_right_pwm(target_pwm)
        self.prev_right_pwm = target_pwm

    def set_left_pwm(self, pwm):
        duty = int(min(abs(pwm), 100) / 100 * MAX_DUTY)
        if pwm >= 0:
            self.pca.channels[LEFT_LPWM_CHANNEL].duty_cycle = duty
            self.pca.channels[LEFT_RPWM_CHANNEL].duty_cycle = 0
        else:
            self.pca.channels[LEFT_LPWM_CHANNEL].duty_cycle = 0
            self.pca.channels[LEFT_RPWM_CHANNEL].duty_cycle = duty

    def set_right_pwm(self, pwm):
        duty = int(min(abs(pwm), 100) / 100 * MAX_DUTY)
        if pwm >= 0:
            self.pca.channels[RIGHT_LPWM_CHANNEL].duty_cycle = duty
            self.pca.channels[RIGHT_RPWM_CHANNEL].duty_cycle = 0
        else:
            self.pca.channels[RIGHT_LPWM_CHANNEL].duty_cycle = 0
            self.pca.channels[RIGHT_RPWM_CHANNEL].duty_cycle = duty

    def soft_start_pwm(self, target_pwm, current_pwm, side):
        step_delay = SOFT_START_DURATION / SOFT_START_STEPS
        for i in range(1, SOFT_START_STEPS + 1):
            interpolated = int(current_pwm + (target_pwm - current_pwm) * i / SOFT_START_STEPS)
            if side == 'left':
                self.set_left_pwm(interpolated)
            elif side == 'right':
                self.set_right_pwm(interpolated)
            time.sleep(step_delay)

def main(args=None):
    rclpy.init(args=args)
    node = MotorController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import Int32
# from adafruit_pca9685 import PCA9685
# import board
# import busio
# import threading
# import time

# # PWM 채널 설정
# LEFT_LPWM_CHANNEL = 0
# LEFT_RPWM_CHANNEL = 1
# RIGHT_LPWM_CHANNEL = 4
# RIGHT_RPWM_CHANNEL = 5

# # EN 핀
# L_LEN = 2
# L_REN = 3
# R_LEN = 6
# R_REN = 7

# PWM_FREQ = 1000
# MAX_DUTY = 0xFFFF

# SOFT_START_THRESHOLD = 30    # 이전값과 30 이상 차이나면 soft start 적용
# SOFT_START_DURATION = 0.2    # soft start 전체 시간 (초)
# SOFT_START_STEPS = 10        # 보간 단계 수

# class MotorController(Node):
#     def __init__(self):
#         super().__init__('motor_controller_node')

#         # I2C 및 PCA9685 초기화
#         i2c = busio.I2C(board.SCL, board.SDA)
#         self.pca = PCA9685(i2c)
#         self.pca.frequency = PWM_FREQ

#         # EN 핀 항상 HIGH
#         for ch in [L_LEN, L_REN, R_LEN, R_REN]:
#             self.pca.channels[ch].duty_cycle = MAX_DUTY

#         # 현재 PWM 상태 저장
#         self.prev_left_pwm = 0
#         self.prev_right_pwm = 0

#         # subscriber
#         self.left_sub = self.create_subscription(Int32, '/left_pwm', self.left_callback, 10)
#         self.right_sub = self.create_subscription(Int32, '/right_pwm', self.right_callback, 10)

#         self.get_logger().info("MotorController node started.")

#     def left_callback(self, msg):
#         target_pwm = msg.data
#         if abs(target_pwm - self.prev_left_pwm) >= SOFT_START_THRESHOLD:
#             threading.Thread(target=self.soft_start_pwm,
#                              args=(target_pwm, self.prev_left_pwm, 'left'), daemon=True).start()
#         else:
#             self.set_left_pwm(target_pwm)
#         self.prev_left_pwm = target_pwm

#     def right_callback(self, msg):
#         target_pwm = msg.data
#         if abs(target_pwm - self.prev_right_pwm) >= SOFT_START_THRESHOLD:
#             threading.Thread(target=self.soft_start_pwm,
#                              args=(target_pwm, self.prev_right_pwm, 'right'), daemon=True).start()
#         else:
#             self.set_right_pwm(target_pwm)
#         self.prev_right_pwm = target_pwm

#     def set_left_pwm(self, pwm):
#         duty = int(min(abs(pwm), 100) / 100 * MAX_DUTY)
#         if pwm >= 0:
#             self.pca.channels[LEFT_LPWM_CHANNEL].duty_cycle = duty
#             self.pca.channels[LEFT_RPWM_CHANNEL].duty_cycle = 0
#         else:
#             self.pca.channels[LEFT_LPWM_CHANNEL].duty_cycle = 0
#             self.pca.channels[LEFT_RPWM_CHANNEL].duty_cycle = duty

#     def set_right_pwm(self, pwm):
#         duty = int(min(abs(pwm), 100) / 100 * MAX_DUTY)
#         if pwm >= 0:
#             self.pca.channels[RIGHT_LPWM_CHANNEL].duty_cycle = duty
#             self.pca.channels[RIGHT_RPWM_CHANNEL].duty_cycle = 0
#         else:
#             self.pca.channels[RIGHT_LPWM_CHANNEL].duty_cycle = 0
#             self.pca.channels[RIGHT_RPWM_CHANNEL].duty_cycle = duty

#     def soft_start_pwm(self, target_pwm, current_pwm, side):
#         step_delay = SOFT_START_DURATION / SOFT_START_STEPS
#         for i in range(1, SOFT_START_STEPS + 1):
#             interpolated = int(current_pwm + (target_pwm - current_pwm) * i / SOFT_START_STEPS)
#             if side == 'left':
#                 self.set_left_pwm(interpolated)
#             elif side == 'right':
#                 self.set_right_pwm(interpolated)
#             time.sleep(step_delay)

# def main(args=None):
#     rclpy.init(args=args)
#     node = MotorController()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()