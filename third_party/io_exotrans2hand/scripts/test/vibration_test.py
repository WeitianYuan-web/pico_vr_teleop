#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import sys

class VibrationPublisher(Node):
    def __init__(self):
        super().__init__('vibration_publisher')

        # 创建发布者
        self.publisher = self.create_publisher(
            Float64MultiArray,
            '/io_esk/vibration_feedback',
            10
        )

        # 创建定时器，3秒
        self.timer = self.create_timer(3.0, self.timer_callback)
        self.counter = 0

        self.get_logger().info('振动发布节点已启动')
        self.get_logger().info('发布话题: /io_teleop/vibration_feedback_exo')

    def timer_callback(self):
        msg = Float64MultiArray()

        # 从0到10逐步提高振动强度
        if self.counter > 10:
            self.get_logger().info('测试完成，停止发布')
            return

        intensity = float(self.counter)
        msg.data = [intensity] * 10
        self.get_logger().info(f'发布: 全部马达强度{self.counter}')

        self.publisher.publish(msg)
        self.counter += 1

def main(args=None):
    rclpy.init(args=args)
    node = VibrationPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('节点停止')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
