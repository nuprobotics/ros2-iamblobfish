#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

class ServiceNode(Node):
    def __init__(self):
        super().__init__('service_node')

        # Declare parameters (service name and default string from config or defaults)
        self.declare_parameter('service_name', '/trigger_service')
        self.declare_parameter('default_string', 'No service available')

        # Retrieve parameters
        self.service_name = self.get_parameter('service_name').get_parameter_value().string_value
        self.default_string = self.get_parameter('default_string').get_parameter_value().string_value
        self.stored_value = self.default_string

        # Create a client to call the /spgc/trigger service
        self.client = self.create_client(Trigger, '/spgc/trigger')

        # Create a service that provides the stored string value
        self.service = self.create_service(Trigger, self.service_name, self.handle_service)

        # Try calling the /spgc/trigger service when starting the node
        self.call_trigger_service()

    def call_trigger_service(self):
        """Call the /spgc/trigger service and store the result."""
        if not self.client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn("Service /spgc/trigger not available, storing default value.")
            self.stored_value = self.default_string
            return

        # Create a request and call the service
        request = Trigger.Request()
        future = self.client.call_async(request)

        # Set up a callback to handle the result
        future.add_done_callback(self.handle_trigger_response)

    def handle_trigger_response(self, future):
        """Handle the response from the /spgc/trigger service."""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f"Received response: {response.message}")
                self.stored_value = response.message
            else:
                self.get_logger().warn("Received unsuccessful response, storing default value.")
                self.stored_value = self.default_string
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")
            self.stored_value = self.default_string

    def handle_service(self, request, response):
        """Handle requests to the provided service by returning the stored value."""
        response.success = True
        response.message = self.stored_value
        self.get_logger().info(f"Returning stored value: {self.stored_value}")
        return response

def main(args=None):
    rclpy.init(args=args)
    node = ServiceNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
