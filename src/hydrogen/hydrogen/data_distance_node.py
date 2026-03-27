#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection2DArray, Detection3DArray, Detection3D, ObjectHypothesisWithPose
from geometry_msgs.msg import Pose, Point, Quaternion

from cv_bridge import CvBridge
import numpy as np
import cv2

from message_filters import Subscriber, ApproximateTimeSynchronizer

from ultralytics.utils.plotting import colors


class ROIDepthFusion(Node):

    def __init__(self):
        super().__init__('roi_depth_fusion_node')

        self.set_parameters([
            Parameter('use_sim_time', Parameter.Type.BOOL, True)
        ])

        # Parameters
        self.declare_parameter('min_depth', 0.2)
        self.declare_parameter('max_depth', 20.0)
        self.declare_parameter('min_valid_pixels', 30)

        self.min_depth = self.get_parameter('min_depth').value
        self.max_depth = self.get_parameter('max_depth').value
        self.min_valid = self.get_parameter('min_valid_pixels').value

        self.bridge = CvBridge()
        self.camera_info = None

        self.class_names = {
            "0": "left_gate_pole",
            "1": "right_gate_pole",
            "2": "shark",
            "3": "sawfish",
            "4": "drop_box",
            "5": "red_buoy",
            "6": "red_pole",
            "7": "white_pole",
            "8": "path_marker",
            "9": "octagon",
            "10": "table",
            "11": "ladle",
            "12": "bottle"
        }

        self.depth_sub = Subscriber(self, Image, '/camera/depth_image_raw/front')
        self.rgb_sub   = Subscriber(self, Image, '/camera/RGB_image_raw/front')
        self.det_sub   = Subscriber(self, Detection2DArray, '/detections_2d')

        self.info_sub = self.create_subscription(
            CameraInfo,
            '/camera_info_front',
            self.camera_info_cb,
            10
        )

        self.sync = ApproximateTimeSynchronizer(
            [self.depth_sub, self.rgb_sub, self.det_sub],
            queue_size=10,
            slop=0.2
        )
        self.sync.registerCallback(self.synced_cb)

        self.pub = self.create_publisher(
            Detection3DArray,
            '/detections_3d',
            10
        )

        
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE
        )
        self.viz_pub = self.create_publisher(Image, '/depth_fusion/detection_image', qos)

        self.get_logger().info("ROI depth fusion node started.")

    def camera_info_cb(self, msg: CameraInfo):
        self.camera_info = msg

    def synced_cb(self, depth_msg: Image,
                  rgb_msg: Image,
                  det_msg: Detection2DArray):

        if self.camera_info is None:
            self.get_logger().warn("Camera info not received yet, skipping processing.")
            return

        depth_img = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='32FC1')
        rgb_img   = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')

        fx = self.camera_info.k[0]
        fy = self.camera_info.k[4]
        cx = self.camera_info.k[2]
        cy = self.camera_info.k[5]

        out = Detection3DArray()
        out.header = det_msg.header

        self.get_logger().info(f"Processing {len(det_msg.detections)} detections.")

        for det in det_msg.detections:
            if not det.results:
                continue

            hyp = det.results[0].hypothesis
            class_id = hyp.class_id
            score = hyp.score

            bbox = det.bbox

            u = int(bbox.center.position.x)
            v = int(bbox.center.position.y)
            w = int(bbox.size_x)
            h = int(bbox.size_y)

            xmin = max(u - w // 2, 0)
            xmax = min(u + w // 2, depth_img.shape[1] - 1)
            ymin = max(v - h // 2, 0)
            ymax = min(v + h // 2, depth_img.shape[0] - 1)

            roi = depth_img[ymin:ymax, xmin:xmax].flatten()
            roi = roi[np.isfinite(roi)]
            roi = roi[(roi > self.min_depth) & (roi < self.max_depth)]

            if roi.size < self.min_valid:
                continue

            z = float(np.median(roi))

            x = (u - cx) * z / fx
            y = (v - cy) * z / fy

            det3d = Detection3D()
            det3d.header = det.header

            hyp3d = ObjectHypothesisWithPose()
            hyp3d.hypothesis.class_id = class_id
            hyp3d.hypothesis.score = score

            pose = Pose()
            pose.position = Point(x=x, y=y, z=z)
            pose.orientation = Quaternion(w=1.0)

            hyp3d.pose.pose = pose
            det3d.results.append(hyp3d)

            det3d.bbox.center = pose
            det3d.bbox.size.x = 0.1
            det3d.bbox.size.y = 0.1
            det3d.bbox.size.z = 0.1

            out.detections.append(det3d)

            color = colors(int(class_id), True)
            cv2.rectangle(rgb_img, (xmin, ymin), (xmax, ymax), color, 2)
            name = self.class_names.get(class_id, class_id)
            label = f"{name} {z:.2f} m"
            cv2.putText(rgb_img, label,
            (xmin, max(ymin - 10, 15)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5, color, 2)

        self.pub.publish(out)

        # rviz publisher
        ros_img = self.bridge.cv2_to_imgmsg(rgb_img, encoding='bgr8')
        ros_img.header = rgb_msg.header
        self.viz_pub.publish(ros_img)
        cv2.imshow("Depth Fusion Detections", rgb_img)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = ROIDepthFusion()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()