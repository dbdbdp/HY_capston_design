from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, TimerAction
import os

def generate_launch_description():
    cart_pkg_share = get_package_share_directory('cartrash')

    # URDF 파일 경로
    urdf_path = os.path.join(cart_pkg_share, 'urdf', 'lidar.urdf')
    with open(urdf_path, 'r') as file:
        robot_description = file.read()

    # 기본 런치 설정
    ld = LaunchDescription([
        # # 기존 노드들
        # Node(
        #     package='robot_control',
        #     executable='motor_controller_node',
        #     name='motor_controller_node',
        #     output='screen'
        # ),
        # Node(
        #     package='robot_control',
        #     executable='encoder_publisher_node',
        #     name='encoder_publisher_node',
        #     output='screen'
        # ),
        # Node(
        #     package='robot_control',
        #     executable='pid_controller_node',
        #     name='pid_controller_node',
        #     output='screen'
        # ),
        # Node(
        #     package='robot_control',
        #     executable='scissor_lift_controller',
        #     name='scissor_lift_controller',
        #     output='screen'
        # ),
        # Node(
        #     package='pump_control',
        #     executable='pump_node',
        #     name='pump_node',
        #     output='screen'
        # ),
        # Node(
        #     package='robot_control',
        #     executable='teleop_keyboard_node2',
        #     name='teleop_keyboard_node2',
        #     output='screen'
        # ),

        # Cartographer 설정 파일 경로
        DeclareLaunchArgument(
            'cartographer_config',
            default_value=os.path.join(cart_pkg_share, 'config', 'rplidar_cartographer.lua'),
            description='Path to Cartographer config file'
        ),
        # URDF 파일 경로
        DeclareLaunchArgument(
            'urdf',
            default_value=urdf_path,
            description='Path to URDF file'
        ),
        # RViz 설정 파일 경로
        DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(cart_pkg_share, 'rviz', 'mapping.rviz'),
            description='Path to RViz config file'
        ),

        # 매핑 및 탐색 관련 노드들
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}]
        ),
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            arguments=[
                '-configuration_directory', os.path.join(cart_pkg_share, 'config'),
                '-configuration_basename', 'rplidar_cartographer.lua'
            ],
            remappings=[('/scan', 'scan')]
        ),
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='occupancy_grid_node',
            output='screen',
            parameters=[{'resolution': 0.05}]
        ),
        Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            name='rplidar_node',
            output='screen',
            parameters=[{
                'serial_port': '/dev/ttyUSB0',
                'serial_baudrate': 115200,
                'frame_id': 'laser_frame',
                'angle_compensate': True,
                'scan_mode': 'Sensitivity'
            }]
        ),
    ])

    # RViz 노드를 5초 후에 실행
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen', 
        arguments=['-d', os.path.join(cart_pkg_share, 'rviz', 'mapping.rviz')]
    )
    ld.add_action(TimerAction(period=5.0, actions=[rviz_node]))

    return ld