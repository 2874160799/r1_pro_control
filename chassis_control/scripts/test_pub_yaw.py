#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import math
import time

class YawPublisher(Node):
    def __init__(self):
        super().__init__('yaw_publisher')
        self.publisher_ = self.create_publisher(Float32, '/YAW_data', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10Hz
        self.start_time = time.time()

    def timer_callback(self):
        msg = Float32()
        # 模拟 yaw 角数据：正弦波变化 0~360
        elapsed = time.time() - self.start_time
        yaw_deg = (math.sin(elapsed) * 180.0)  # -180 ~ 180
        msg.data = yaw_deg
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing yaw: {yaw_deg:.2f} deg')

def main(args=None):
    rclpy.init(args=args)
    node = YawPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
