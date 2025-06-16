from setuptools import setup
import os
from glob import glob

package_name = 'robot_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='teamleafia',
    maintainer_email='teamleafia@todo.todo',
    description='Motor control and encoder feedback for plant transport robot.',
    license='MIT',
    tests_require=['pytest'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # launch 파일 전체 복사
        #('share/' + package_name + '/launch', glob('launch/*.py')),
        # config 파일 전체 복사
        #('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    entry_points={
        'console_scripts': [
            'motor_controller_node = robot_control.motor_controller_node:main',
            'pid_controller_node = robot_control.pid_controller_node:main',
            'encoder_publisher_node = robot_control.encoder_publisher_node:main',
            #'cmd_vel_to_wheel_vel_node = robot_control.cmd_vel_to_wheel_vel_node:main',
            #'odometry_node = robot_control.odometry_node:main',
            #'fake_encoder_publisher_node = robot_control.fake_encoder_publisher_node:main',
            'scissor_lift_controller = robot_control.scissor_lift_controller:main',
            #'movement_command_node = robot_control.movement_command_node:main',
            'teleop_keyboard_node = robot_control.teleop_keyboard_node:main',
            #'gpiod_test = robot_control.gpiod_test:main',
            'move_distance_node=robot_control.move_distance_node:main',
            'teleop_keyboard_node2 = robot_control.teleop_keyboard_node2:main',
            'teleop_keyboard_node4 = robot_control.teleop_keyboard_node4:main',

        ],
    },
)
