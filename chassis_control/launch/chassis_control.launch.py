from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='chassis_control',
            executable='chassis_control_node',  # 你的 CMake 里 install 的可执行文件名
            name='chassis_control',
            output='screen',
            parameters=[
                {"imu_topic": "/IMU_data"},
                {"yaw_topic": "/YAW_data"},
                {"queue_size": 10}
            ]
        )
    ])
