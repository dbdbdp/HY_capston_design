from setuptools import find_packages, setup

package_name = 'aruco_detector'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='teamleafia',
    maintainer_email='teamleafia@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "aruco_detector=aruco_detector.aruco_detector:main",
            "aruco_move_node=aruco_detector.aruco_move_node:main",
            "aruco_detector_node=aruco_detector.aruco_detector_node:main",
        ],
    },
)