#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class ObjectSubscriber(Node):
    def __init__(self):
        super().__init__('object_subscriber')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            String, '/detected_objects', self.detection_callback, 10)
        self.subscription_ = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
        
    def image_callback(self, img):
        self.frame = self.bridge.imgmsg_to_cv2(img, 'bgr8')

    def detection_callback(self, msg):
        data = msg.data
        self.get_logger().info(f"Received message: {data}")

        # Parsing the string data (split by " + ")
        parsed_data = data.split(" + ")

        # Extracting class name and bounding box coordinates
        class_name = parsed_data[0]
        x1 = float(parsed_data[1])
        y1 = float(parsed_data[2])
        x3 = float(parsed_data[3])
        y3 = float(parsed_data[4])
        x2 = float(parsed_data[5])
        y2 = float(parsed_data[6])
        x4 = float(parsed_data[7])
        y4 = float(parsed_data[8])
        self.get_logger().info(f"Received class: {class_name}")
        # Drawing the bounding box on the image
        top_left = (int(x1), int(y1))  # Top-left corner of the bounding box
        bottom_right = (int(x2), int(y2))  # Bottom-right corner of the bounding box
        color = (0, 255, 0)  # Green color for the bounding box
        thickness = 2  # Line thickness

        # Draw the rectangle (bounding box)
        try:
            cv2.rectangle(self.frame, top_left, bottom_right, color, thickness)

            # Optionally, draw the class name on top of the bounding box
            font = cv2.FONT_HERSHEY_SIMPLEX
            cv2.putText(self.frame, class_name, (int(x1), int(y1) - 10), font, 0.9, (255, 0, 0), 2)

            # Display the image with bounding boxes
            cv2.imshow('Detection', self.frame)
            cv2.waitKey(1)  # Wait for 1 ms to update the image window
        except:
            pass

def main(args=None):
    rclpy.init(args=args)
    node = ObjectSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()