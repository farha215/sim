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

class VisionFusionNode(Node):
    """
    Node for 3D object detection by fusing YOLO 2D detections with depth map data.
    """
    def __init__(self):
        super().__init__('vision_fusion_node')

        self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])

        # Parameters
        self.declare_parameter('min_depth', 0.2)
        self.declare_parameter('max_depth', 20.0)
        self.declare_parameter('min_valid_pixels', 10)
        self.declare_parameter('gate_conf', 0.6)
        self.declare_parameter('pole_conf', 0.3)

        self.min_depth = self.get_parameter('min_depth').value
        self.max_depth = self.get_parameter('max_depth').value
        self.min_valid = self.get_parameter('min_valid_pixels').value
        self.gate_conf = self.get_parameter('gate_conf').value
        self.pole_conf = self.get_parameter('pole_conf').value

        self.bridge = CvBridge()
        self.camera_info = None

        # Load YOLO Model
        package_share_directory = get_package_share_directory('hydrogen')
        model_path = os.path.join(package_share_directory, 'prequal.pt')
        self.model = YOLO(model_path)
        self.get_logger().info(f"YOLO model loaded from {model_path}")

        # Subscribers
        self.depth_sub = Subscriber(self, Image, '/camera/depth_image_raw/front')
        self.rgb_sub   = Subscriber(self, Image, '/camera/RGB_image_raw/front')
        
        self.info_sub = self.create_subscription(CameraInfo, '/camera_info_front', self.camera_info_cb, 10)

        self.sync = ApproximateTimeSynchronizer([self.depth_sub, self.rgb_sub], queue_size=10, slop=0.1)
        self.sync.registerCallback(self.synced_cb)

        # Publisher
        self.pub = self.create_publisher(Detection3DArray, '/detections_3d', 10)

        self.last_proc_time = self.get_clock().now()
        self.get_logger().info("Vision Fusion Node initialized.")

    def camera_info_cb(self, msg: CameraInfo):
        self.camera_info = msg

    def synced_cb(self, depth_msg: Image, rgb_msg: Image):
        # Throttle to ~15Hz to save CPU
        now = self.get_clock().now()
        if (now - self.last_proc_time).nanoseconds < 66000000:
            return
        self.last_proc_time = now

        if self.camera_info is None:
            return

        # 1. Image Conversion
        cv_image = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
        depth_img = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='32FC1')

        # 2. YOLO Inference
        results = self.model(cv_image, verbose=False, show=False)

        # 3. Detection Processing
        out = Detection3DArray()
        out.header = rgb_msg.header

        fx = self.camera_info.k[0]
        fy = self.camera_info.k[4]
        cx = self.camera_info.k[2]
        cy = self.camera_info.k[5]

        if len(results) > 0:
            for box in results[0].boxes:
                conf = float(box.conf[0].cpu().numpy())
                cls = int(box.cls[0].cpu().numpy())
                cls_name = self.model.names.get(cls, str(cls))

                # Confidence Filtering
                if cls_name == "preq_gate" and conf < self.gate_conf: continue
                if cls_name == "preq_pole" and conf < self.pole_conf: continue

                # Bounding Box Extraction
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                u, v = int((x1 + x2) / 2.0), int((y1 + y2) / 2.0)

                # 4. Depth Fusion (ROI Median)
                xmin, xmax = max(int(x1), 0), min(int(x2), depth_img.shape[1] - 1)
                ymin, ymax = max(int(y1), 0), min(int(y2), depth_img.shape[0] - 1)

                roi = depth_img[ymin:ymax, xmin:xmax].flatten()
                roi = roi[np.isfinite(roi)]
                roi = roi[(roi > self.min_depth) & (roi < self.max_depth)]

                if roi.size < self.min_valid:
                    continue

                z = float(np.median(roi))
                x = (u - cx) * z / fx
                y = (v - cy) * z / fy

                # 5. Construct 3D Detection
                det3d = Detection3D()
                det3d.header = out.header

                hyp3d = ObjectHypothesisWithPose()
                hyp3d.hypothesis.class_id = "preq_gate" if cls_name == "preq_gate" else ("preq_pole" if cls_name == "preq_pole" else cls_name)
                hyp3d.hypothesis.score = conf

                pose = Pose()
                pose.position = Point(x=x, y=y, z=z)
                pose.orientation = Quaternion(w=1.0)

                hyp3d.pose.pose = pose
                det3d.results.append(hyp3d)
                det3d.bbox.center = pose
                det3d.bbox.size.x = det3d.bbox.size.y = det3d.bbox.size.z = 0.1

                out.detections.append(det3d)

                # --- 6. Visualization ---
                label = f"{hyp3d.hypothesis.class_id}: {z:.2f}m"
                cv2.rectangle(cv_image, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                cv2.putText(cv_image, label, (int(x1), int(y1) - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Show visualization window
        cv2.imshow("Vision Detections", cv_image)
        cv2.waitKey(1)

        self.pub.publish(out)

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
