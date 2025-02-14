from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='swerve_controller',
            executable='swerve_serial_publisher',
            name='swerve_serial_publisher',
            output='screen'
        )
    ])
