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

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        corners, ids, _ = cv2.aruco.detectMarkers(cv_image, self.aruco_dict, parameters=self.parameters)
        if ids is not None:
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, self.marker_size, self.camera_matrix, self.dist_coeffs)
            for i in range(len(ids)):
                distance = np.linalg.norm(tvecs[i])-0.015
                corner = corners[i][0]
                cx = float(np.mean(corner[:,0]))
                rmat, _ = cv2.Rodrigues(rvecs[i])
                yaw = np.arctan2(rmat[1, 0], rmat[0, 0])
                pitch = np.arctan2(-rmat[2, 0], np.sqrt(rmat[2, 1]**2 + rmat[2, 2]**2))
                roll = np.arctan2(rmat[2, 1], rmat[2, 2])
                marker_info = f"ID:{ids[i]},cx:{cx:.1f}px,Dist:{distance:.3f}m,Yaw:{np.degrees(yaw):.2f}°,Pitch:{np.degrees(pitch):.2f}°,Roll:{np.degrees(roll):.2f}°"
                self.marker_publisher.publish(String(data=marker_info))
                self.get_logger().info(marker_info)

                # 거리 발행
                self.distance_publisher.publish(Float32(data=distance))
                self.cx_publisher.publish(Float32(data=cx))
                self.pitch_publisher.publish(Float32(data=np.degrees(pitch)))

                cv2.drawFrameAxes(cv_image, self.camera_matrix, self.dist_coeffs, rvecs[i], tvecs[i], 0.05)

        cv2.imshow("ArUco_detection", cv_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    aruco_detector = ArucoDetector()
    rclpy.spin(aruco_detector)
    aruco_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


#ls /dev/ttyUSB*
#ros2 run usb_cam usb_cam_node_exe --ros-args -p video_device:="/dev/video4"
#ros2 run usb_cam usb_cam_node_exe


# ros2 run usb_cam usb_cam_node_exe --ros-args -p frame_rate:=15.0
