from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='motor_publisher',
            executable='motorpublisher',
            output='screen'),
    ])

