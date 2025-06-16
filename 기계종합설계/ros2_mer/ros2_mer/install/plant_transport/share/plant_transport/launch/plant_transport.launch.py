from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # ArUco Detector 노드
        Node(
            package='aruco_detector',
            executable='aruco_detector',
            name='aruco_detector'
        ),
        # ArUco Move Node (거리 추출 및 발행)
        # Node(
        #     package='aruco_detector',
        #     executable='aruco_move_node',
        #     name='aruco_move_node'
        # ),
        # Motor Controller 노드
        Node(
            package='robot_control',
            executable='motor_controller_node',
            name='motor_controller_node'
        ),
        
        Node(
        package='robot_control',  # 노드가 속한 패키지
        executable='move_distance_node',
        name='move_distance_node'
        ),
        Node(
        package='robot_control',  # 노드가 속한 패키지
        executable='encoder_publisher_node',
        name='encoder_publisher_node'
        ),
        Node(
        package='robot_control',  # 노드가 속한 패키지
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
    ])