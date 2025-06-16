import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from adafruit_pca9685 import PCA9685
import board
import busio
import gpiod
import threading
import time

# ===== 설정 =====
PCA_CHANNEL = 8
PWM_FREQ = 1000
DIR_OFFSET = 12
CHIP_NAME = "gpiochip4"
MAX_TRAVEL_TIME = 3.0  # 3초

class ScissorLiftController(Node):
    def __init__(self):
        super().__init__('scissor_lift_controller')

        # PWM 초기화
        i2c = busio.I2C(board.SCL, board.SDA)
        self.pca = PCA9685(i2c)
        self.pca.frequency = PWM_FREQ

        # DIR 제어 초기화
        self.chip = gpiod.Chip(CHIP_NAME)
        self.dir_line = self.chip.get_line(DIR_OFFSET)
        self.dir_line.request(consumer="scissor_lift", type=gpiod.LINE_REQ_DIR_OUT)

        # 토픽
        self.subscription = self.create_subscription(
            String,
            '/scissor_lift_cmd',
            self.cmd_callback,
            10
        )
        self.lift_done_pub = self.create_publisher(Bool, 'lift_done', 10)

        self.lock = threading.Lock()
        self.active_timer = None
        self.publish_lift_done = False

        self.get_logger().info("Scissor Lift Controller Node with STOP callback triggered after 2 sec.")

    def cmd_callback(self, msg):
        command = msg.data.lower().strip()

        with self.lock:
            if self.active_timer:
                self.active_timer.cancel()
                self.active_timer = None

        if command == "down":
            self.set_dir(0)
            self.set_pwm(0.8)
            self.publish_lift_done = False
            self.start_safety_timer()
        elif command == "up":
            self.set_dir(1)
            self.set_pwm(0.8)
            self.publish_lift_done = True
            self.start_safety_timer()
        elif command == "stop":
            self.set_pwm(0.0)
            if self.publish_lift_done:
                msg = Bool()
                msg.data = True
                self.lift_done_pub.publish(msg)
                self.get_logger().info("Published 'lift_done: True'")
                self.publish_lift_done = False
        else:
            self.get_logger().warn(f"Unknown command: '{command}'")

    def set_pwm(self, duty_ratio):
        pwm_val = int(min(max(duty_ratio, 0.0), 1.0) * 0xFFFF)
        self.pca.channels[PCA_CHANNEL].duty_cycle = pwm_val

    def set_dir(self, level: int):
        self.dir_line.set_value(level)

    def start_safety_timer(self):
        with self.lock:
            self.active_timer = threading.Timer(MAX_TRAVEL_TIME, self.inject_stop_command)
            self.active_timer.start()

    def inject_stop_command(self):
        # 내부적으로 "stop" 명령을 처리 (토픽 발행 아님)
        self.get_logger().info("MAX_TRAVEL_TIME reached → sending internal 'stop' command")
        self.cmd_callback(String(data="stop"))

    def destroy_node(self):
        self.set_pwm(0.0)
        if self.dir_line:
            self.dir_line.set_value(0)
            self.dir_line.release()
        self.get_logger().info("Scissor Lift Node stopped and GPIO released.")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ScissorLiftController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
