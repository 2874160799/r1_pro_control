from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='track',
            executable='track_node',
            name='track_node',
            output='screen',
            parameters=[
                {'device_leftfront':0},
                {'hmin': 0},
                {'hmax': 180},
                {'smin': 0},
                {'smax': 255},
                {'lmin': 0},
                {'lmax': 130}
                ]
        )
    ])