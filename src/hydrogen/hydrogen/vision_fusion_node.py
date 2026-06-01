#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection3DArray, Detection3D, ObjectHypothesisWithPose
from geometry_msgs.msg import Pose, Point, Quaternion
from cv_bridge import CvBridge
import numpy as np
import cv2
import os
from ultralytics import YOLO
from ament_index_python.packages import get_package_share_directory
from message_filters import Subscriber, ApproximateTimeSynchronizer

# ── HSV pole detection constants (Red) ───────────────────────────────────────
# Red spans both ends of the H spectrum (0-180 in OpenCV)
RED_HSV_LO1 = np.array([170, 50, 30], dtype=np.uint8)
RED_HSV_HI1 = np.array([179, 255, 255], dtype=np.uint8)
RED_HSV_LO2 = np.array([0, 50, 30], dtype=np.uint8)
RED_HSV_HI2 = np.array([10, 255, 255], dtype=np.uint8)

# Morphology: dilate to connect any small gaps in the thin pole
DILATE_KERNEL = np.ones((9, 9), np.uint8)
DILATE_ITERS  = 2

# Detection thresholds
MIN_CONTOUR_AREA = 100
POLE_ASPECT_THRESHOLD = 1.5  # h/w ratio for pole discrimination

class VisionFusionNode(Node):
    """
    Node for 3D object detection by fusing YOLO 2D detections (gate)
    and HSV-based 2D detections (pole) with depth map data.
    """
    def __init__(self):
        super().__init__('vision_fusion_node')

        self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])

        # Parameters
        self.declare_parameter('min_depth', 0.2)
        self.declare_parameter('max_depth', 20.0)
        self.declare_parameter('min_valid_pixels', 10)
        self.declare_parameter('gate_conf', 0.8)

        self.min_depth = self.get_parameter('min_depth').value
        self.max_depth = self.get_parameter('max_depth').value
        self.min_valid = self.get_parameter('min_valid_pixels').value
        self.gate_conf = self.get_parameter('gate_conf').value

        self.bridge = CvBridge()
        self.camera_info = None

        # Load YOLO Model for Gate
        package_share_directory = get_package_share_directory('hydrogen')
        model_path = os.path.join(package_share_directory, 'prequal.pt')
        self.model = YOLO(model_path)
        self.get_logger().info(f"YOLO model loaded for gate detection from {model_path}")

        # Subscribers
        self.depth_sub = Subscriber(self, Image, '/camera/depth_image_raw/front')
        self.rgb_sub   = Subscriber(self, Image, '/camera/RGB_image_raw/front')
        
        self.info_sub = self.create_subscription(CameraInfo, '/camera_info_front', self.camera_info_cb, 10)

        self.sync = ApproximateTimeSynchronizer([self.depth_sub, self.rgb_sub], queue_size=10, slop=0.1)
        self.sync.registerCallback(self.synced_cb)

        # Publisher
        self.pub = self.create_publisher(Detection3DArray, '/detections_3d', 10)

        self.last_proc_time = self.get_clock().now()
        self.get_logger().info("Vision Fusion Node initialized (Hybrid: YOLO Gate + HSV Pole).")

    def camera_info_cb(self, msg: CameraInfo):
        self.camera_info = msg

    def get_z_from_depth(self, depth_img, xmin, ymin, xmax, ymax, is_pole=False):
        """
        Extracts median depth from an ROI, with optional foreground clustering for thin poles.
        """
        roi = depth_img[ymin:ymax, xmin:xmax].flatten()
        roi = roi[np.isfinite(roi)]
        roi = roi[(roi > self.min_depth) & (roi < self.max_depth)]

        if roi.size < self.min_valid:
            return None

        if is_pole:
            # For thin poles, cluster nearest pixels to avoid background contamination
            z_min = roi.min()
            foreground = roi[roi < z_min + 0.5]
            z = float(np.median(foreground)) if foreground.size >= 5 else float(np.median(roi))
        else:
            z = float(np.median(roi))
        
        return z

    def synced_cb(self, depth_msg: Image, rgb_msg: Image):
        # Throttle processing to save CPU
        now = self.get_clock().now()
        if (now - self.last_proc_time).nanoseconds < 66000000: # ~15Hz
            return
        self.last_proc_time = now

        if self.camera_info is None:
            return

        # 1. Image Conversion
        rgb_cv = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
        depth_cv = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='32FC1')

        out = Detection3DArray()
        out.header = rgb_msg.header

        fx = self.camera_info.k[0]
        fy = self.camera_info.k[4]
        cx = self.camera_info.k[2]
        cy = self.camera_info.k[5]

        # ── 2. Gate Detection (YOLO) ─────────────────────────────────────────
        results = self.model(rgb_cv, verbose=False, show=False)

        if len(results) > 0:
            for box in results[0].boxes:
                conf = float(box.conf[0].cpu().numpy())
                cls = int(box.cls[0].cpu().numpy())
                cls_name = self.model.names.get(cls, str(cls))

                # We only want the gate from the model
                if cls_name != "preq_gate" or conf < self.gate_conf:
                    continue

                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                
                # Depth extraction
                z = self.get_z_from_depth(depth_cv, int(x1), int(y1), int(x2), int(y2), is_pole=False)
                if z is None: continue

                # Back-projection to 3D
                u_center, v_center = (x1 + x2) / 2.0, (y1 + y2) / 2.0
                x_3d = (u_center - cx) * z / fx
                y_3d = (v_center - cy) * z / fy

                # Add to output
                self.add_detection_3d(out, "preq_gate", conf, x_3d, y_3d, z, (x2-x1), (y2-y1))

                # Visualization
                cv2.rectangle(rgb_cv, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                cv2.putText(rgb_cv, f"gate: {z:.2f}m", (int(x1), int(y1) - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # ── 3. Pole Detection (HSV - Red) ────────────────────────────────────
        hsv = cv2.cvtColor(rgb_cv, cv2.COLOR_BGR2HSV)
        mask1 = cv2.inRange(hsv, RED_HSV_LO1, RED_HSV_HI1)
        mask2 = cv2.inRange(hsv, RED_HSV_LO2, RED_HSV_HI2)
        mask = cv2.bitwise_or(mask1, mask2)
        mask = cv2.dilate(mask, DILATE_KERNEL, iterations=DILATE_ITERS)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Sort contours by area, largest first
        contours = sorted(contours, key=cv2.contourArea, reverse=True)

        for cnt in contours:
            if cv2.contourArea(cnt) < MIN_CONTOUR_AREA:
                continue

            x, y, w, h = cv2.boundingRect(cnt)
            
            # Simple aspect ratio filter to ensure it's a vertical pole
            if h / max(w, 1) < POLE_ASPECT_THRESHOLD:
                continue

            # Depth extraction
            z = self.get_z_from_depth(depth_cv, x, y, x + w, y + h, is_pole=True)
            if z is None: continue

            # Back-projection to 3D
            u_center, v_center = x + w / 2.0, y + h / 2.0
            x_3d = (u_center - cx) * z / fx
            y_3d = (v_center - cy) * z / fy

            # Add to output
            self.add_detection_3d(out, "preq_pole", 1.0, x_3d, y_3d, z, float(w), float(h))

            # Visualization
            cv2.rectangle(rgb_cv, (x, y), (x + w, y + h), (0, 0, 255), 2)
            cv2.putText(rgb_cv, f"pole: {z:.2f}m", (x, y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            
            # Only track the single largest valid pole to prevent jumping/flickering
            break

        # 4. Final Publish and Visualise
        cv2.imshow("Vision Detections (Hybrid)", rgb_cv)
        cv2.waitKey(1)
        self.pub.publish(out)

    def add_detection_3d(self, out_array, class_id, score, x, y, z, w_px, h_px):
        det3d = Detection3D()
        det3d.header = out_array.header

        hyp3d = ObjectHypothesisWithPose()
        hyp3d.hypothesis.class_id = class_id
        hyp3d.hypothesis.score = score

        pose = Pose()
        pose.position = Point(x=float(x), y=float(y), z=float(z))
        pose.orientation = Quaternion(w=1.0)

        hyp3d.pose.pose = pose
        det3d.results.append(hyp3d)
        det3d.bbox.center = pose
        det3d.bbox.size.x = det3d.bbox.size.y = det3d.bbox.size.z = 0.1 # Placeholder size

        out_array.detections.append(det3d)

def main(args=None):
    rclpy.init(args=args)
    node = VisionFusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
