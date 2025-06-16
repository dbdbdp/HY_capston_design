# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import String, Float32

# class ArucoMoveNode(Node):
#     def __init__(self):
#         super().__init__('aruco_move_node')
#         # /aruco_markers 토픽 구독
#         self.subscription = self.create_subscription(
#             String,
#             '/aruco_markers',
#             self.aruco_callback,
#             10)
#         # /move_distance 토픽 발행
#         self.publisher = self.create_publisher(Float32, '/move_distance', 10)
#         self.get_logger().info("ArucoMoveNode started.")

#     def aruco_callback(self, msg):
#         # 메시지에서 Dist 값 추출
#         parts = msg.data.split(',')
#         for part in parts:
#             if 'Dist:' in part:
#                 dist_str = part.split(':')[1]  # "Dist:0.500" -> "0.500"
#                 distance = float(dist_str)     # 문자열을 float로 변환
#                 # /move_distance에 발행
#                 self.publisher.publish(Float32(data=distance))
#                 self.get_logger().info(f'발행한 거리: {distance:.3f} m')
#                 break

# def main(args=None):
#     rclpy.init(args=args)
#     node = ArucoMoveNode()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()