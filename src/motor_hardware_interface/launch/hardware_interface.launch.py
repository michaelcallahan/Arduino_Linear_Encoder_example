import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python import get_package_share_directory

def generate_launch_description():
    urdf_file_name = 'linear_cnc_stage.urdf'
    urdf = os.path.join(
        get_package_share_directory('motor_hardware_interface'),
        'urdf',
        urdf_file_name)

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time if true'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')},
                        {'robot_description': open(urdf).read()}]
        ),

        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[os.path.join(
                get_package_share_directory('motor_hardware_interface'),
                'config',
                'controller_config.yaml')],
            output='screen'
        )
    ])
