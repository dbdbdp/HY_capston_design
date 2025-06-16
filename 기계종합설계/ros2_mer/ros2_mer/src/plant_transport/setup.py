from setuptools import setup

package_name = 'plant_transport'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/plant_transport.launch.py']),
        ('share/' + package_name + '/launch', ['launch/experiment_launch.py']),
        ('share/' + package_name + '/launch', ['launch/plant_teleop_key_launch.py']),
        ('share/' + package_name + '/launch', ['launch/plant_cart_tel_key_launch.py']),
        ('share/' + package_name + '/launch', ['launch/total_launch.py']),
        ('share/' + package_name + '/launch', ['launch/yj_align_launch.py']),
        ('share/' + package_name + '/launch', ['launch/total2_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='teamleafia',
    maintainer_email='teamleafia@todo.todo',
    description='Integrated package for plant transport robot',
    license='Apache-2.0',
    tests_require=['pytest'],
)