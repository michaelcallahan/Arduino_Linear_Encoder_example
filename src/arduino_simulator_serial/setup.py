from setuptools import find_packages, setup

package_name = 'arduino_simulator_serial'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'python-serial'],
    zip_safe=True,
    maintainer='michaelcallahan',
    author_email='mike.w.callahan@gmail.com',
    description='Arduino Simulator Node for ROS2 using Virtual Serial Port',
    license='MIT License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'arduino_simulator_node = arduino_simulator_serial.arduino_simulator_node:main',
        ],
    },
)
