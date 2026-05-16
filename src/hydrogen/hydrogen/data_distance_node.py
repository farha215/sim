#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection3DArray, Detection3D, ObjectHypothesisWithPose
from geometry_msgs.msg import Pose, Point, Quaternion

from cv_bridge import CvBridge
import numpy as np
import cv2

from message_filters import Subscriber, ApproximateTimeSynchronizer


# ── HSV colour ranges for the orange gate and pole ───────────────────────────
# Both objects are the same orange/yellow colour.
# Sampled from Gazebo sim screenshots: H≈8-19, S≈60-255, V≈60-255.
# Wide V floor (60) catches the pole when it is partially shadowed or far away.
ORANGE_HSV_LO = np.array([5,  60,  60], dtype=np.uint8)
ORANGE_HSV_HI = np.array([35, 255, 255], dtype=np.uint8)

# Morphology: dilate thin structures before contour detection.
# 9×9 kernel, 2 iterations works for the thin vertical pole seen at ~5 m.
DILATE_KERNEL    = np.ones((9, 9), np.uint8)
DILATE_ITERS     = 2

# Contour filtering
MIN_CONTOUR_AREA = 200     # px² after dilation — rejects tiny noise specks
MIN_VALID_DEPTH  = 20      # pixels with valid depth required inside ROI

# Aspect-ratio gate/pole discriminator.
# Gate bbox is wide (aspect < 2).  Pole bbox is tall (aspect ≥ 2).
POLE_ASPECT_THRESHOLD = 2.0   # h/w ≥ this → preq_pole, else → preq_gate


class ROIDepthFusion(Node):

    def __init__(self):
        super().__init__('roi_depth_fusion_node')

        self.set_parameters([
            Parameter('use_sim_time', Parameter.Type.BOOL, True)
        ])

        self.declare_parameter('min_depth', 0.2)
        self.declare_parameter('max_depth', 20.0)
        self.declare_parameter('min_valid_pixels', MIN_VALID_DEPTH)

        self.min_depth = self.get_parameter('min_depth').value
        self.max_depth = self.get_parameter('max_depth').value
        self.min_valid = self.get_parameter('min_valid_pixels').value

        self.bridge       = CvBridge()
        self.camera_info  = None

        # Synchronise depth + RGB only (no YOLO detection topic needed)
        self.depth_sub = Subscriber(self, Image, '/camera/depth_image_raw/front')
        self.rgb_sub   = Subscriber(self, Image, '/camera/RGB_image_raw/front')

        self.info_sub = self.create_subscription(
            CameraInfo,
            '/camera_info_front',
            self.camera_info_cb,
            10
        )

        self.sync = ApproximateTimeSynchronizer(
            [self.depth_sub, self.rgb_sub],
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
        self.viz_pub = self.create_publisher(
            Image, '/depth_fusion/detection_image', qos
        )

        self.get_logger().info(
            "ROI depth fusion node started (OpenCV HSV detector)."
        )

    # ── Camera info ───────────────────────────────────────────────────────────
    def camera_info_cb(self, msg: CameraInfo):
        self.camera_info = msg

    # ── Main callback ─────────────────────────────────────────────────────────
    def synced_cb(self, depth_msg: Image, rgb_msg: Image):

        if self.camera_info is None:
            self.get_logger().warn(
                "Camera info not received yet, skipping.", throttle_duration_sec=2.0
            )
            return

        depth_img = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='32FC1')
        rgb_img   = self.bridge.imgmsg_to_cv2(rgb_msg,   desired_encoding='bgr8')

        fx = self.camera_info.k[0]
        fy = self.camera_info.k[4]
        cx = self.camera_info.k[2]
        cy = self.camera_info.k[5]

        # ── Step 1: HSV mask ─────────────────────────────────────────────────
        hsv  = cv2.cvtColor(rgb_img, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, ORANGE_HSV_LO, ORANGE_HSV_HI)

        # ── Step 2: Inflate thin structures ──────────────────────────────────
        mask_dilated = cv2.dilate(mask, DILATE_KERNEL, iterations=DILATE_ITERS)

        # ── Step 3: Contour detection ─────────────────────────────────────────
        contours, _ = cv2.findContours(
            mask_dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        out = Detection3DArray()
        out.header = rgb_msg.header

        viz = rgb_img.copy()

        for contour in contours:
            area = cv2.contourArea(contour)
            if area < MIN_CONTOUR_AREA:
                continue

            x, y, w, h = cv2.boundingRect(contour)

            # Clamp to image bounds
            xmin = max(x, 0)
            ymin = max(y, 0)
            xmax = min(x + w, depth_img.shape[1] - 1)
            ymax = min(y + h, depth_img.shape[0] - 1)

            # ── Step 4: Depth extraction from ROI ────────────────────────────
            roi = depth_img[ymin:ymax, xmin:xmax].flatten()
            roi = roi[np.isfinite(roi)]
            roi = roi[(roi > self.min_depth) & (roi < self.max_depth)]

            if roi.size < self.min_valid:
                continue

            # Foreground clustering: keep pixels within 0.5 m of the nearest
            # valid pixel — avoids background contamination on thin objects.
            z_min      = roi.min()
            foreground = roi[roi < z_min + 0.5]
            z = float(np.median(foreground)) if foreground.size >= 5 \
                else float(np.median(roi))

            # Pixel centre of bounding box → camera-frame X, Y
            u = xmin + w // 2
            v = ymin + h // 2
            cam_x = (u - cx) * z / fx
            cam_y = (v - cy) * z / fy

            # ── Step 5: Gate vs Pole discriminator ───────────────────────────
            aspect    = h / max(w, 1)
            class_id  = "preq_pole" if aspect >= POLE_ASPECT_THRESHOLD \
                        else "preq_gate"

            # Only detect gate if it is within 2 meters
            if class_id == "preq_gate" and z > 2.0:
                continue

            # ── Build Detection3D ─────────────────────────────────────────────
            det3d        = Detection3D()
            det3d.header = rgb_msg.header

            hyp3d = ObjectHypothesisWithPose()
            hyp3d.hypothesis.class_id = class_id
            hyp3d.hypothesis.score    = 1.0   # OpenCV doesn't give a score

            pose              = Pose()
            pose.position     = Point(x=cam_x, y=cam_y, z=z)
            pose.orientation  = Quaternion(w=1.0)
            hyp3d.pose.pose   = pose

            det3d.results.append(hyp3d)

            det3d.bbox.center   = pose
            det3d.bbox.size.x   = float(w)
            det3d.bbox.size.y   = float(h)
            det3d.bbox.size.z   = 0.1

            out.detections.append(det3d)

            self.get_logger().info(
                f"[{class_id}] bbox=({xmin},{ymin},{w}x{h}) "
                f"aspect={aspect:.1f} z={z:.2f} m  cam=({cam_x:.2f},{cam_y:.2f})"
            )

            # ── Visualisation overlay ─────────────────────────────────────────
            color = (0, 165, 255) if class_id == "preq_gate" else (0, 255, 0)
            cv2.rectangle(viz, (xmin, ymin), (xmax, ymax), color, 2)
            cv2.putText(
                viz, f"{class_id} {z:.2f}m",
                (xmin, max(ymin - 10, 15)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2
            )

        self.pub.publish(out)

        ros_img        = self.bridge.cv2_to_imgmsg(viz, encoding='bgr8')
        ros_img.header = rgb_msg.header
        self.viz_pub.publish(ros_img)

        cv2.imshow("Depth Fusion Detections", viz)
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