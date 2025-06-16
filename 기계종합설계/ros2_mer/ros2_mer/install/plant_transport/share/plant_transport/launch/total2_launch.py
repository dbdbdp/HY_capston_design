from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # ArUco Detector YJ 노드 (OpenCV 표시 포함)
        Node(
            package='aruco_detector',
            executable='aruco_detector_node',
            name='aruco_detector_node',
            output='screen',
        ),
        
        # Motor Controller 노드
        Node(
            package='robot_control',
            executable='motor_controller_node',
            name='motor_controller_node'
        ),
        
        # Encoder Publisher 노드
        Node(
            package='robot_control',
            executable='encoder_publisher_node',
            name='encoder_publisher_node'
        ),
        
        # PID Controller 노드
        Node(
            package='robot_control',
            executable='pid_controller_node',
            name='pid_controller_node'
        ),
        
        # Scissor Lift Controller 노드
        Node(
            package='robot_control',
            executable='scissor_lift_controller',
            name='scissor_lift_controller'
        ),
        
        # Pump Control 노드
        Node(
            package='pump_control',
            executable='pump_node',
            name='pump_node'
        ),
        
        # Marker Align 노드
        Node(
            package='marker_align_2',
            executable='marker_align_node2',
            name='marker_align_node2'
        ),
        
        # # Teleop Keyboard 노드
        # Node(
        #     package='robot_control',
        #     executable='teleop_keyboard_node4',
        #     name='teleop_keyboard_node4'
        # ),
    ])