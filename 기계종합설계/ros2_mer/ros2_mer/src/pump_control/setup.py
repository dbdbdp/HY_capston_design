from setuptools import setup

package_name = 'pump_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='teamleafia',
    maintainer_email='teamleafia@todo.todo',
    description='Pump control node',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pump_node = pump_control.pump_node:main',
            'pump_testnode = pump_control.pump_testnode:main',
        ],
    },
)

