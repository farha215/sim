#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Float32


class PressureConverter(Node):
    """Converts /pressure_raw (Float64, from the depth plugin) to /pressure (Float32)."""

    def __init__(self):
        super().__init__('pressure_converter')

        self.pub = self.create_publisher(Float32, '/pressure', 10)
        self.create_subscription(Float64, '/pressure_raw', self.on_raw, 10)

    def on_raw(self, msg: Float64):
        out = Float32()
        out.data = float(msg.data)
        self.pub.publish(out)


def main():
    rclpy.init()
    node = PressureConverter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
