#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy

from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose


class DetectionTestPublisher(Node):

    def __init__(self):
        super().__init__('detection_test_publisher')

        self.pub = self.create_publisher(
            Detection2DArray,
            '/detections_2d',
            10
        )

        self.timer = self.create_timer(0.5, self.timer_cb)
        self.get_logger().info('Detection2D test publisher started')

    def timer_cb(self):
        msg = Detection2DArray()

        # This now uses Gazebo sim time via /clock bridge
        now = self.get_clock().now().to_msg()

        msg.header.stamp = now
        msg.header.frame_id = 'zed_camera_front_link'

        det = Detection2D()
        det.header.stamp = now
        det.header.frame_id = 'zed_camera_front_link'

        det.bbox.center.position.x = 320.0
        det.bbox.center.position.y = 240.0
        det.bbox.center.theta = 0.0
        det.bbox.size_x = 100.0
        det.bbox.size_y = 80.0

        hyp = ObjectHypothesisWithPose()
        hyp.hypothesis.class_id = 'test'
        hyp.hypothesis.score = 0.95

        det.results.append(hyp)
        msg.detections.append(det)

        self.pub.publish(msg)
        self.get_logger().info(
            f'Published fake detection at t={now.sec}.{now.nanosec:09d}'
        )


def main():
    rclpy.init()
    node = DetectionTestPublisher()

    # Enable sim time so get_clock() follows /clock
    node.set_parameters([
        Parameter('use_sim_time', Parameter.Type.BOOL, True)
    ])

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()