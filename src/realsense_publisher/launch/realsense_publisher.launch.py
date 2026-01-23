from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='realsense_d455_publisher',
            executable='realsense_publisher',
            name='realsense_publisher',
            output='screen',
            parameters=[{
                'width': 640,
                'height': 480,
                'fps': 30
            }]
        )
    ])