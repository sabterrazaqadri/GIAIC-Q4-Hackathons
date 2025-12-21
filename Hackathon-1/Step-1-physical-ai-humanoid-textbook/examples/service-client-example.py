#!/usr/bin/env python3
"""
Service Client Example for ROS 2 Fundamentals Module

This node implements a simple service client that requests the current time
from a service server. It demonstrates the client-service pattern in ROS 2.
"""

import rclpy
from rclpy.node import Node
from example_interfaces.srv import Trigger
import sys


class TimeServiceClient(Node):
    """
    A simple ROS 2 service client that requests the current time from a service.
    """

    def __init__(self):
        # Initialize the node with the name 'time_service_client'
        super().__init__('time_service_client')
        
        # Create a client for the 'get_current_time' service
        # The service uses the Trigger message type
        self.cli = self.create_client(Trigger, 'get_current_time')
        
        # Wait for the service to be available
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        
        # Create a request object
        self.request = Trigger.Request()

    def send_request(self):
        """
        Send a request to the service and return the future for the response.
        """
        # Call the service asynchronously
        self.future = self.cli.call_async(self.request)
        return self.future


def main(args=None):
    """
    Main function to initialize the ROS 2 client library, create the service client,
    send a request, and wait for the response.
    """
    # Initialize the ROS 2 client library
    rclpy.init(args=args)

    # Create an instance of the TimeServiceClient class
    time_service_client = TimeServiceClient()

    # Send a request to the service
    future = time_service_client.send_request()

    # Keep the node running until the response is received
    try:
        # Wait for the result
        rclpy.spin_until_future_complete(time_service_client, future)
        
        # Process the response
        if future.result() is not None:
            response = future.result()
            time_service_client.get_logger().info(
                f'Response received - Success: {response.success}, Message: {response.message}')
        else:
            time_service_client.get_logger().info('Service call failed')
            
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up and shut down the node
        time_service_client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()