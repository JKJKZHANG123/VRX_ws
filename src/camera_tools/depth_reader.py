#!/usr/bin/env python3
"""读取奥比中光 Astra Pro Plus 深度摄像头中心点的距离"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np

class DepthReader(Node):
    def __init__(self):
        super().__init__('depth_reader')
        self.sub = self.create_subscription(
            Image, '/camera/depth/image_raw', self.callback, 10)
        self.get_logger().info('监听 /camera/depth/image_raw ...')

    def callback(self, msg):
        h, w = msg.height, msg.width
        # Y11 格式：每个像素 2 字节，小端序
        data = np.frombuffer(msg.data, dtype=np.uint16).reshape(h, w)

        # 取中心点距离（单位：毫米）
        cx, cy = w // 2, h // 2
        dist_mm = data[cy, cx]
        dist_m = dist_mm / 1000.0

        self.get_logger().info(
            f'中心点 ({cx},{cy}) 距离: {dist_mm} mm = {dist_m:.3f} m')

        # 也可以扫一整行看距离分布
        row = data[cy, :]
        valid = row[row > 0]
        if len(valid) > 0:
            self.get_logger().info(
                f'整行有效距离: min={valid.min()}mm, max={valid.max()}mm, '
                f'avg={valid.mean():.0f}mm')

def main():
    rclpy.init()
    node = DepthReader()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
