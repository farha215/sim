#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2

from message_filters import Subscriber, ApproximateTimeSynchronizer


class ROIDepthManualDebug(Node):

    def __init__(self):
        super().__init__('roi_depth_manual_debug')

        # Use Gazebo sim time
        self.set_parameters([
            Parameter('use_sim_time', Parameter.Type.BOOL, True)
        ])

        self.bridge = CvBridge()

        # ROI drawing state
        self.roi = None
        self.drawing = False
        self.start_pt = None

        # Subscribers
        self.depth_sub = Subscriber(self, Image, '/camera/depth_image_raw/front')
        self.rgb_sub   = Subscriber(self, Image, '/camera/RGB_image_raw/front')

        self.sync = ApproximateTimeSynchronizer(
            [self.depth_sub, self.rgb_sub],
            queue_size=10,
            slop=0.2
        )
        self.sync.registerCallback(self.synced_cb)

        cv2.namedWindow("ROI Depth Debug")
        cv2.setMouseCallback("ROI Depth Debug", self.mouse_cb)

        self.get_logger().info("Manual ROI depth debug node started.")
        self.get_logger().info("Draw ROI with left mouse button.")

    def mouse_cb(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.drawing = True
            self.start_pt = (x, y)
            self.roi = None

        elif event == cv2.EVENT_MOUSEMOVE and self.drawing:
            self.roi = (self.start_pt[0], self.start_pt[1], x, y)

        elif event == cv2.EVENT_LBUTTONUP:
            self.drawing = False
            self.roi = (self.start_pt[0], self.start_pt[1], x, y)

    def synced_cb(self, depth_msg: Image, rgb_msg: Image):

        depth_img = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
        if depth_img.dtype == np.uint16:
            depth_img = depth_img.astype(np.float32) * 0.001  # mm → m

        rgb_img = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')

        if self.roi is not None:
            x1, y1, x2, y2 = self.roi
            xmin, xmax = sorted([x1, x2])
            ymin, ymax = sorted([y1, y2])

            xmin = max(0, xmin)
            ymin = max(0, ymin)
            xmax = min(rgb_img.shape[1] - 1, xmax)
            ymax = min(rgb_img.shape[0] - 1, ymax)

            roi_depth = depth_img[ymin:ymax, xmin:xmax].flatten()
            roi_depth = roi_depth[np.isfinite(roi_depth)]
            roi_depth = roi_depth[roi_depth > 0.0]

            if roi_depth.size > 0:
                z = float(np.median(roi_depth))
                label = f"{z:.2f} m"
                cv2.putText(rgb_img, label,
                            (xmin, max(ymin - 10, 0)),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.6, (0, 0, 255), 2)

            cv2.rectangle(rgb_img, (xmin, ymin), (xmax, ymax),
                          (0, 255, 0), 2)

        cv2.imshow("ROI Depth Debug", rgb_img)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = ROIDepthManualDebug()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
