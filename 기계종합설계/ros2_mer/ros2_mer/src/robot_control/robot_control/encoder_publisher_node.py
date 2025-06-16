import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import gpiod
import threading
import time

CHIP_NAME = 'gpiochip4'
LEFT_ENC_A_OFFSET = 17
LEFT_ENC_B_OFFSET = 22
RIGHT_ENC_A_OFFSET = 27
RIGHT_ENC_B_OFFSET = 23
CPR = 348  # 모터 스펙에 따라 조정

class EncoderPublisher(Node):
    def __init__(self):
        super().__init__('encoder_publisher_node')
        self.left_pub = self.create_publisher(Int32, '/left_encoder', 10)
        self.right_pub = self.create_publisher(Int32, '/right_encoder', 10)
        self.left_rps_pub = self.create_publisher(Int32, '/left_rps', 10)
        self.right_rps_pub = self.create_publisher(Int32, '/right_rps', 10)

        self.left_count = 0
        self.right_count = 0
        self.prev_left_count = 0
        self.prev_right_count = 0

        self.chip = gpiod.Chip(CHIP_NAME)
        self.left_a = self.chip.get_line(LEFT_ENC_A_OFFSET)
        self.left_b = self.chip.get_line(LEFT_ENC_B_OFFSET)
        self.right_a = self.chip.get_line(RIGHT_ENC_A_OFFSET)
        self.right_b = self.chip.get_line(RIGHT_ENC_B_OFFSET)

        self.left_a.request(consumer="encoder", type=gpiod.LINE_REQ_EV_BOTH_EDGES)
        self.left_b.request(consumer="encoder", type=gpiod.LINE_REQ_DIR_IN)
        self.right_a.request(consumer="encoder", type=gpiod.LINE_REQ_EV_BOTH_EDGES)
        self.right_b.request(consumer="encoder", type=gpiod.LINE_REQ_DIR_IN)

        threading.Thread(target=self.left_callback, daemon=True).start()
        threading.Thread(target=self.right_callback, daemon=True).start()

        self.create_timer(0.02, self.publish_data)  # 50 Hz

    def left_callback(self):
        while True:
            if self.left_a.event_wait(sec=1):
                event = self.left_a.event_read()
                b_state = self.left_b.get_value()
                if event.type == gpiod.LineEvent.RISING_EDGE:
                    direction = 1 if b_state == 0 else -1
                else:  # FALLING_EDGE
                    direction = -1 if b_state == 0 else 1
                self.left_count -= direction  # 방향 반전

    def right_callback(self):
        while True:
            if self.right_a.event_wait(sec=1):
                event = self.right_a.event_read()
                b_state = self.right_b.get_value()
                if event.type == gpiod.LineEvent.RISING_EDGE:
                    direction = 1 if b_state == 0 else -1
                else:
                    direction = -1 if b_state == 0 else 1
                self.right_count -= direction  # 방향 반전

    def publish_data(self):
        self.left_pub.publish(Int32(data=self.left_count))
        self.right_pub.publish(Int32(data=self.right_count))

        left_rps = (self.left_count - self.prev_left_count) * 50 / CPR
        right_rps = (self.right_count - self.prev_right_count) * 50 / CPR

        self.left_rps_pub.publish(Int32(data=int(left_rps)))
        self.right_rps_pub.publish(Int32(data=int(right_rps)))

        self.prev_left_count = self.left_count
        self.prev_right_count = self.right_count

def main(args=None):
    rclpy.init(args=args)
    node = EncoderPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


#### ###
# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import Int32
# import gpiod
# import threading
# import time

# CHIP_NAME = 'gpiochip4'
# LEFT_ENC_A_OFFSET = 17
# LEFT_ENC_B_OFFSET = 22
# RIGHT_ENC_A_OFFSET = 27
# RIGHT_ENC_B_OFFSET = 23
# CPR = 348  # 모터 스펙에 따라 조정

# class EncoderPublisher(Node):
#     def __init__(self):
#         super().__init__('encoder_publisher_node')
#         self.left_pub = self.create_publisher(Int32, '/left_encoder', 10)
#         self.right_pub = self.create_publisher(Int32, '/right_encoder', 10)
#         self.left_rps_pub = self.create_publisher(Int32, '/left_rps', 10)
#         self.right_rps_pub = self.create_publisher(Int32, '/right_rps', 10)

#         self.left_count = 0
#         self.right_count = 0
#         self.prev_left_count = 0
#         self.prev_right_count = 0

#         self.chip = gpiod.Chip(CHIP_NAME)
#         self.left_a = self.chip.get_line(LEFT_ENC_A_OFFSET)
#         self.left_b = self.chip.get_line(LEFT_ENC_B_OFFSET)
#         self.right_a = self.chip.get_line(RIGHT_ENC_A_OFFSET)
#         self.right_b = self.chip.get_line(RIGHT_ENC_B_OFFSET)

#         self.left_a.request(consumer="encoder", type=gpiod.LINE_REQ_EV_BOTH_EDGES)
#         self.left_b.request(consumer="encoder", type=gpiod.LINE_REQ_DIR_IN)
#         self.right_a.request(consumer="encoder", type=gpiod.LINE_REQ_EV_BOTH_EDGES)
#         self.right_b.request(consumer="encoder", type=gpiod.LINE_REQ_DIR_IN)

#         threading.Thread(target=self.left_callback, daemon=True).start()
#         threading.Thread(target=self.right_callback, daemon=True).start()

#         self.create_timer(0.02, self.publish_data)  # 50 Hz

#     def left_callback(self):
#         while True:
#             if self.left_a.event_wait(sec=1):
#                 event = self.left_a.event_read()
#                 b_state = self.left_b.get_value()
#                 if event.type == gpiod.LineEvent.RISING_EDGE:
#                     direction = 1 if b_state == 0 else -1
#                 else:  # FALLING_EDGE
#                     direction = -1 if b_state == 0 else 1
#                 self.left_count -= direction  # 방향 반전

#     def right_callback(self):
#         while True:
#             if self.right_a.event_wait(sec=1):
#                 event = self.right_a.event_read()
#                 b_state = self.right_b.get_value()
#                 if event.type == gpiod.LineEvent.RISING_EDGE:
#                     direction = 1 if b_state == 0 else -1
#                 else:
#                     direction = -1 if b_state == 0 else 1
#                 self.right_count -= direction  # 방향 반전

#     def publish_data(self):
#         self.left_pub.publish(Int32(data=self.left_count))
#         self.right_pub.publish(Int32(data=self.right_count))

#         left_rps = (self.left_count - self.prev_left_count) * 50 / CPR
#         right_rps = (self.right_count - self.prev_right_count) * 50 / CPR

#         self.left_rps_pub.publish(Int32(data=int(left_rps)))
#         self.right_rps_pub.publish(Int32(data=int(right_rps)))

#         self.prev_left_count = self.left_count
#         self.prev_right_count = self.right_count

#         self.get_logger().info(
#             f'L_cnt: {self.left_count} | R_cnt: {self.right_count} | '
#             f'L_rps: {left_rps:.2f} | R_rps: {right_rps:.2f}'
#         )

# def main(args=None):
#     rclpy.init(args=args)
#     node = EncoderPublisher()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

####@@

# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import Int32
# import gpiod
# import threading
# import time

# CHIP_NAME = 'gpiochip4'
# LEFT_ENC_A_OFFSET = 17
# LEFT_ENC_B_OFFSET = 22
# RIGHT_ENC_A_OFFSET = 27
# RIGHT_ENC_B_OFFSET = 23
# CPR = 348  # 모터 스펙에 따라 조정

# class EncoderPublisher(Node):
#     def __init__(self):
#         super().__init__('encoder_publisher_node')
#         self.left_pub = self.create_publisher(Int32, '/left_encoder', 10)
#         self.right_pub = self.create_publisher(Int32, '/right_encoder', 10)
#         self.left_rps_pub = self.create_publisher(Int32, '/left_rps', 10)
#         self.right_rps_pub = self.create_publisher(Int32, '/right_rps', 10)

#         self.left_count = 0
#         self.right_count = 0
#         self.prev_left_count = 0
#         self.prev_right_count = 0

#         self.chip = gpiod.Chip(CHIP_NAME)
#         self.left_a = self.chip.get_line(LEFT_ENC_A_OFFSET)
#         self.left_b = self.chip.get_line(LEFT_ENC_B_OFFSET)
#         self.right_a = self.chip.get_line(RIGHT_ENC_A_OFFSET)
#         self.right_b = self.chip.get_line(RIGHT_ENC_B_OFFSET)

#         self.left_a.request(consumer="encoder", type=gpiod.LINE_REQ_EV_BOTH_EDGES)
#         self.left_b.request(consumer="encoder", type=gpiod.LINE_REQ_DIR_IN)
#         self.right_a.request(consumer="encoder", type=gpiod.LINE_REQ_EV_BOTH_EDGES)
#         self.right_b.request(consumer="encoder", type=gpiod.LINE_REQ_DIR_IN)

#         threading.Thread(target=self.left_callback, daemon=True).start()
#         threading.Thread(target=self.right_callback, daemon=True).start()

#         self.create_timer(0.02, self.publish_data)  # 50 Hz

#     def left_callback(self):
#         while True:
#             if self.left_a.event_wait(sec=1):
#                 event = self.left_a.event_read()
#                 b_state = self.left_b.get_value()
#                 if event.type == gpiod.LineEvent.RISING_EDGE:
#                     direction = 1 if b_state == 0 else -1
#                 else:  # FALLING_EDGE
#                     direction = -1 if b_state == 0 else 1
#                 self.left_count += direction

#     def right_callback(self):
#         while True:
#             if self.right_a.event_wait(sec=1):
#                 event = self.right_a.event_read()
#                 b_state = self.right_b.get_value()
#                 if event.type == gpiod.LineEvent.RISING_EDGE:
#                     direction = 1 if b_state == 0 else -1
#                 else:
#                     direction = -1 if b_state == 0 else 1
#                 self.right_count += direction

#     def publish_data(self):
#         self.left_pub.publish(Int32(data=self.left_count))
#         self.right_pub.publish(Int32(data=self.right_count))

#         left_rps = (self.left_count - self.prev_left_count) * 50 / CPR
#         right_rps = (self.right_count - self.prev_right_count) * 50 / CPR

#         self.left_rps_pub.publish(Int32(data=int(left_rps)))
#         self.right_rps_pub.publish(Int32(data=int(right_rps)))

#         self.prev_left_count = self.left_count
#         self.prev_right_count = self.right_count

#         self.get_logger().info(
#             f'L_cnt: {self.left_count} | R_cnt: {self.right_count} | '
#             f'L_rps: {left_rps:.2f} | R_rps: {right_rps:.2f}'
#         )

# def main(args=None):
#     rclpy.init(args=args)
#     node = EncoderPublisher()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()