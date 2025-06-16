import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String, Float32, Bool
from cv_bridge import CvBridge
import cv2
import numpy as np
from cv2 import aruco

class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')
        self.subscription = self.create_subscription(
            Image, '/image_raw', self.image_callback, 10)
        self.parking_possible_sub = self.create_subscription(
            Bool, '/show_start_ui', self.parking_possible_callback, 10)
        self.bridge = CvBridge()
        self.marker_publisher = self.create_publisher(String, '/aruco_markers', 10)
        self.distance_publisher = self.create_publisher(Float32, '/move_distance', 10)
        self.cx_publisher = self.create_publisher(Float32, '/marker_cx', 10)
        self.pitch_publisher = self.create_publisher(Float32, '/marker_pitch', 10)

        # ArUco 설정
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.parameters = cv2.aruco.DetectorParameters()
        self.camera_matrix = np.array([[577.111712, 0., 349.233035],
                                       [0., 578.640658, 234.939843],
                                       [0., 0., 1.]], dtype=np.float32)
        self.dist_coeffs = np.array([-0.445927, 0.185899, -0.001744, -0.003167, 0.000000], dtype=np.float32)
        self.marker_size = 0.08

        # 주차 가능 상태
        self.parking_possible = False

    def parking_possible_callback(self, msg):
        self.parking_possible = msg.data

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        corners, ids, _ = cv2.aruco.detectMarkers(cv_image, self.aruco_dict, parameters=self.parameters)
        distance = None
        pitch_deg = None

        if ids is not None:
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, self.marker_size, self.camera_matrix, self.dist_coeffs)
            for i in range(len(ids)):
                distance = np.linalg.norm(tvecs[i]) - 0.015
                corner = corners[i][0]
                cx = float(np.mean(corner[:,0]))
                rmat, _ = cv2.Rodrigues(rvecs[i])
                yaw = np.arctan2(rmat[1, 0], rmat[0, 0])
                pitch = np.arctan2(-rmat[2, 0], np.sqrt(rmat[2, 1]**2 + rmat[2, 2]**2))
                pitch_deg = np.degrees(pitch)
                roll = np.arctan2(rmat[2, 1], rmat[2, 2])
                marker_info = f"ID:{ids[i]},cx:{cx:.1f}px,Dist:{distance:.3f}m,Yaw:{np.degrees(yaw):.2f}°,Pitch:{pitch_deg:.2f}°,Roll:{np.degrees(roll):.2f}°"
                self.marker_publisher.publish(String(data=marker_info))
                self.get_logger().info(marker_info)

                # 발행
                self.distance_publisher.publish(Float32(data=float(distance)))
                self.cx_publisher.publish(Float32(data=cx))
                self.pitch_publisher.publish(Float32(data=pitch_deg))

                # 마커 및 좌표축
                cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)
                cv2.drawFrameAxes(cv_image, self.camera_matrix, self.dist_coeffs, rvecs[i], tvecs[i], 0.05)

        # 텍스트 오버레이
        y_offset = 30
        cv2.putText(
            cv_image,
            f"Distance: {distance:.3f} m" if distance is not None else "Distance: N/A",
            (10, y_offset),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 0),
            1,
            cv2.LINE_AA
        )
        y_offset += 30
        cv2.putText(
            cv_image,
            f"Pitch: {pitch_deg:.2f} deg" if pitch_deg is not None else "Pitch: N/A",
            (10, y_offset),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 0),
            1,
            cv2.LINE_AA
        )
        y_offset += 30
        cv2.putText(
            cv_image,
            f"Parking Possible: {'Yes' if self.parking_possible else 'No'}",
            (10, y_offset),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 0) if self.parking_possible else (0, 0, 255),
            1,
            cv2.LINE_AA
        )

        # OpenCV 윈도우
        cv2.imshow("ArUco_detection", cv_image)
        cv2.waitKey(1)

    def destroy_node(self):
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