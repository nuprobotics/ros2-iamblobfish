#!/usr/bin/env python3

import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_srvs.srv import Trigger
from cv_bridge import CvBridge
import cv2

class ImageSaverNode(Node):
    def __init__(self):
        super().__init__('image_saver')

        # Declare the topic name as a parameter from the command line
        self.declare_parameter('image_topic', '/camera/image/compressed')
        self.image_topic = self.get_parameter('image_topic').get_parameter_value().string_value

        # Create a CvBridge instance to convert ROS images to OpenCV format
        self.bridge = CvBridge()

        # Variable to determine if we should save images
        self.saving_enabled = True

        # Create a subscriber for the image topic
        self.subscription = self.create_subscription(
            CompressedImage,
            self.image_topic,
            self.image_callback,
            10
        )

        # Provide a service to stop saving images
        self.stop_service = self.create_service(
            Trigger,
            f'/I_LOVE_ILIA/stop',
            self.handle_stop_service
        )

        # Create output directory for saving images
        self.output_dir = 'output_images'
        os.makedirs(self.output_dir, exist_ok=True)

        self.get_logger().info(f"Subscribed to {self.image_topic} and saving images to {self.output_dir}")

    def image_callback(self, msg):
        """Callback to handle received images and save them to disk."""
        if self.saving_enabled:
            try:
                # Convert the compressed image to OpenCV format
                cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='passthrough')

                # Create a unique filename and save the image
                image_filename = os.path.join(self.output_dir, f'image_{self.get_clock().now().to_msg().sec}.jpg')
                cv2.imwrite(image_filename, cv_image)
                self.get_logger().info(f"Image saved: {image_filename}")
            except Exception as e:
                self.get_logger().error(f"Failed to save image: {e}")
        else:
            self.get_logger().info("Image saving is disabled.")

    def handle_stop_service(self, request, response):
        """Service callback to stop saving images."""
        self.saving_enabled = False
        self.get_logger().info("Image saving has been stopped.")
        response.success = True
        response.message = "Image saving stopped."
        return response

def main(args=None):
    rclpy.init(args=args)
    node = ImageSaverNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
