import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String, Float32
from cv_bridge import CvBridge
import cv2
import numpy as np
from cv2 import aruco

class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')
        self.subscription = self.create_subscription(
            Image, '/image_raw', self.image_callback, 10)
        self.bridge = CvBridge()
        self.marker_publisher = self.create_publisher(String, '/aruco_markers', 10)
        self.distance_publisher = self.create_publisher(Float32, '/move_distance', 10)

        # ArUco 설정
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.parameters = cv2.aruco.DetectorParameters()
        self.camera_matrix = np.array([[577.111712, 0., 349.233035],
                                       [0., 578.640658, 234.939843],
                                       [0., 0., 1.]], dtype=np.float32)
        self.dist_coeffs = np.array([-0.445927, 0.185899, -0.001744, -0.003167, 0.000000], dtype=np.float32)
        self.marker_size = 0.08

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        corners, ids, rejected = cv2.aruco.detectMarkers(cv_image, self.aruco_dict, parameters=self.parameters)

        if ids is not None:
            # 마커 그리기
            cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)

            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, self.marker_size, self.camera_matrix, self.dist_coeffs)
            for i in range(len(ids)):
                distance = np.linalg.norm(tvecs[i])-0.015
                rmat, _ = cv2.Rodrigues(rvecs[i])
                yaw = np.arctan2(rmat[1, 0], rmat[0, 0])
                pitch = np.arctan2(-rmat[2, 0], np.sqrt(rmat[2, 1]**2 + rmat[2, 2]**2))
                roll = np.arctan2(rmat[2, 1], rmat[2, 2])
                marker_info = f"ID:{ids[i]},Dist:{distance:.3f},Yaw:{np.degrees(yaw):.2f},Pitch:{np.degrees(pitch):.2f},Roll:{np.degrees(roll):.2f}"
                self.marker_publisher.publish(String(data=marker_info))

                # 거리 정보 텍스트를 이미지에 표시
                text = f"ID: {ids[i][0]}, Dist: {distance:.3f}m, Pitch:{np.degrees(pitch):.2f}"
                # 마커의 첫 번째 코너 위치 근처에 텍스트 표시
                text_position = (int(corners[i][0][0][0]), int(corners[i][0][0][1] - 10))
                cv2.putText(
                    cv_image,
                    text,
                    text_position,
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,  # 폰트 크기
                    (0, 255, 0),  # 초록색 텍스트
                    1,  # 두께
                    cv2.LINE_AA
                )

                # 거리 발행
                distance_msg = Float32()
                distance_msg.data = float(distance)
                self.distance_publisher.publish(distance_msg)

        # OpenCV 윈도우에 이미지 표시
        cv2.imshow("ArUco Detection", cv_image)
        cv2.waitKey(1)

    def destroy_node(self):
        # 노드 종료 시 OpenCV 윈도우 닫기
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    aruco_detector = ArucoDetector()
    try:
        rclpy.spin(aruco_detector)
    except KeyboardInterrupt:
        pass
    finally:
        aruco_detector.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# from std_msgs.msg import String, Float32
# from cv_bridge import CvBridge
# import cv2
# import numpy as np
# from cv2 import aruco

# class ArucoDetector(Node):
#     def __init__(self):
#         super().__init__('aruco_detector')
#         self.subscription = self.create_subscription(
#             Image, '/image_raw', self.image_callback, 10)
#         self.bridge = CvBridge()
#         self.marker_publisher = self.create_publisher(String, '/aruco_markers', 10)
#         self.distance_publisher = self.create_publisher(Float32, '/move_distance', 10)

#         # ArUco 설정
#         self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
#         self.parameters = cv2.aruco.DetectorParameters()
#         self.camera_matrix = np.array([[556.149826, 0., 345.353610],
#                                        [0., 557.965707, 229.537301],
#                                        [0., 0., 1.]], dtype=np.float32)
#         self.dist_coeffs = np.array([-0.431385, 0.156131, 0.000488, 0.000288, 0.000000], dtype=np.float32)
#         self.marker_size = 0.08

#     def image_callback(self, msg):
#         cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
#         corners, ids, _ = cv2.aruco.detectMarkers(cv_image, self.aruco_dict, parameters=self.parameters)
#         if ids is not None:
#             rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, self.marker_size, self.camera_matrix, self.dist_coeffs)
#             for i in range(len(ids)):
#                 distance = np.linalg.norm(tvecs[i])
#                 rmat, _ = cv2.Rodrigues(rvecs[i])
#                 yaw = np.arctan2(rmat[1, 0], rmat[0, 0])
#                 pitch = np.arctan2(-rmat[2, 0], np.sqrt(rmat[2, 1]**2 + rmat[2, 2]**2))
#                 roll = np.arctan2(rmat[2, 1], rmat[2, 2])
#                 marker_info = f"ID:{ids[i]},Dist:{distance:.3f},Yaw:{np.degrees(yaw):.2f},Pitch:{np.degrees(pitch):.2f},Roll:{np.degrees(roll):.2f}"
#                 self.marker_publisher.publish(String(data=marker_info))
#                 #self.get_logger().info(marker_info)

#                 # 거리 발행
#                 distance_msg = Float32()
#                 distance_msg.data = float(distance)
#                 self.distance_publisher.publish(distance_msg)

#         # cv2.imshow("ArUco_detection", cv_image)
#         # cv2.waitKey(1)

# def main(args=None):
#     rclpy.init(args=args)
#     aruco_detector = ArucoDetector()
#     rclpy.spin(aruco_detector)
#     aruco_detector.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()