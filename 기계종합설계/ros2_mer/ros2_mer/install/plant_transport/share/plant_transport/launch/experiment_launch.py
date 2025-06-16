from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
       
        Node(
            package='robot_control',
            executable='motor_controller_node',
            name='motor_controller_node'
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
    ])