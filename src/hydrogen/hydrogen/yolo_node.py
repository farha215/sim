#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
from ultralytics import YOLO
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from ament_index_python.packages import get_package_share_directory


class ImageCollector(Node):
    def __init__(self):
        super().__init__('image_collector')

        self.topic_name = '/camera/RGB_image_raw/front'
        self.save_path = os.path.expanduser('~/auv_dataset/images')

        self.subscription = self.create_subscription(
            Image,
            self.topic_name,
            self.listener_callback,
            10)

        self.bridge = CvBridge()

        # Dynamic path to the YOLO model
        package_share_directory = get_package_share_directory('hydrogen')
        model_path = os.path.join(package_share_directory, 'best.pt')
        self.model = YOLO(model_path)

        if not os.path.exists(self.save_path):
            os.makedirs(self.save_path)
            self.get_logger().info(f"Created folder: {self.save_path}")

   
        self.image_pub = self.create_publisher(Image, '/yolo/detection_image', 10)

        # 2d array publisher for distance node 
        self.detection_pub = self.create_publisher(Detection2DArray, '/detections_2d', 10)

        self.counter = 0
        self.get_logger().info(f"Subscribed to {self.topic_name}. Ready to process images...")

    def listener_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            results = self.model(cv_image)

            det_array = Detection2DArray()
            det_array.header = msg.header

            for result in results:
                for box in result.boxes:
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                    conf = float(box.conf[0].cpu().numpy())
                    cls = int(box.cls[0].cpu().numpy())

                    det = Detection2D()
                    det.header = msg.header

                    det.bbox.center.position.x = float((x1 + x2) / 2.0)
                    det.bbox.center.position.y = float((y1 + y2) / 2.0)
                    det.bbox.size_x = float(x2 - x1)
                    det.bbox.size_y = float(y2 - y1)

                    hyp = ObjectHypothesisWithPose()
                    hyp.hypothesis.class_id = str(cls)
                    hyp.hypothesis.score = conf
                    det.results.append(hyp)

                    det_array.detections.append(det)

            self.detection_pub.publish(det_array)

            annotated_frame = results[0].plot()
            ros_image = self.bridge.cv2_to_imgmsg(annotated_frame, encoding="bgr8")
            ros_image.header = msg.header
            self.image_pub.publish(ros_image)

            self.counter += 1

        except Exception as e:
            self.get_logger().error(f'Error: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = ImageCollector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()