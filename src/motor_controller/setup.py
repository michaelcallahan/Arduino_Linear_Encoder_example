from setuptools import find_packages, setup

package_name = 'motor_controller'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='michaelcallahan',
    description='ROS2 Motor Controller Node',
    license='MIT License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor_controller_node = motor_controller.motor_controller_node:main',
        ],
    },
)
