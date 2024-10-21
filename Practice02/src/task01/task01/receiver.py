#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ReceiverNode(Node):
    def __init__(self):
        super().__init__('receiver')  # Node name is 'receiver'
        self.subscription = self.create_subscription(
            String,
            '/spgc/sender',  # Topic name
            self.listener_callback,
            10  # QoS profile
        )
        self.subscription  # Prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f"{msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = ReceiverNode()
    try:
        rclpy.spin(node)  # Keep the node alive
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
