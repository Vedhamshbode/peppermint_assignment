#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, ObjectHypothesisWithPose
import cv2
import numpy as np
from ultralytics import YOLO
from std_msgs.msg import String
import math

class ObjectDetection(Node):
    def __init__(self):
        super().__init__('object_detection')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
        self.publisher = self.create_publisher(String, '/detected_objects', 10)

        # Load YOLO
        self.model = YOLO("yolo11n.pt")

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            height, width, channels = frame.shape

            results = self.model(frame)
            for result in results:
                for box in result.boxes:
        # Extract bounding box coordinates
                    x1, y1, x2, y2 = box.xyxy[0].tolist()  # Top-left and bottom-right coordinates
                    confidence = box.conf.item()  # Confidence score
                    class_id = box.cls.item()  # Class ID
                    class_name = self.model.names[class_id]
                    h = math.sqrt((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1))
                    # x3 = x1 + h/math.sqrt(2)
                    x3 = x2
                    x4 = x1
                    y3 = y1
                    y4 = y2
                    img = String()
                    img.data = f"{class_name} + {x1} + {y1} + {x3} + {y3} + {x2} + {y2} + {x4} + {y4}"
                    print(img)
            # print(class_name, x1, y1, x2, y2)
            self.publisher.publish(img)

        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetection()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()