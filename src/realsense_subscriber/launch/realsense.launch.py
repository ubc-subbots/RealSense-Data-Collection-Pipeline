# realsense.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        
        Node(
            package='realsense_publisher',
            executable='realsense_publisher',
            name='realsense_publisher',
            parameters=[{
                'width': 640,
                'height': 480,
                'fps': 30,
                'enable_imu': True,
            }]
        ),

        Node(
            package='realsense_subscriber',
            executable='realsense_subscriber',
            name='realsense_yolo_subscriber',
            parameters=[{
                'model_path': '/home/eraofcoding/Subbots/deepsense_test/src/realsense_subscriber/realsense_subscriber/best.pt',
                'confidence_threshold': 0.5,
                'device': 'cpu',
                'save_images': True,
            }]
        ),

    ])