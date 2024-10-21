#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class PublisherNode(Node):
    def __init__(self):
        super().__init__('publisher')

        # Declare parameters
        self.declare_parameter('text', 'Hello, ROS2!')
        self.declare_parameter('topic_name', '/spgc/receiver')

        # Get the topic name from parameters (from config or default)
        topic_name = self.get_parameter('topic_name').get_parameter_value().string_value

        # Create the publisher
        self.publisher_ = self.create_publisher(String, topic_name, 10)

        # Create a timer that will trigger the publishing
        timer_period = 1.0  # Publish every 1 second
        self.timer = self.create_timer(timer_period, self.publish_message)

        # Store the message to publish
        self.message = String()

    def publish_message(self):
        # Get the text from the command-line parameter (or use default)
        text_param = self.get_parameter('text').get_parameter_value().string_value
        self.message.data = text_param
        self.publisher_.publish(self.message)
        self.get_logger().info(f'Publishing: "{self.message.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = PublisherNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
