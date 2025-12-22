#!/usr/bin/env python3
"""
Service Server Example for ROS 2 Fundamentals Module

This node implements a simple service server that provides the current time
when requested. It demonstrates the service-server pattern in ROS 2.
"""

import rclpy
from rclpy.node import Node
import time
from example_interfaces.srv import Trigger


class TimeServiceServer(Node):
    """
    A simple ROS 2 service server that returns the current time when requested.
    """

    def __init__(self):
        # Initialize the node with the name 'time_service_server'
        super().__init__('time_service_server')
        
        # Create a service that responds to requests on the 'get_current_time' topic
        # The service uses the Trigger message type, which has no request fields
        # and returns a boolean success value with a message
        self.srv = self.create_service(
            Trigger,
            'get_current_time',
            self.time_callback)
        
        # Log that the service server has started
        self.get_logger().info('Time service server initialized')

    def time_callback(self, request, response):
        """
        This method is called whenever a request is received for the service.
        It returns the current time in the response.
        """
        # Get the current time
        current_time = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
        
        # Set the response
        response.success = True
        response.message = f"Current time is: {current_time}"
        
        # Log that a request was processed
        self.get_logger().info(f'Handled time request, returning: {response.message}')
        
        # Return the response
        return response


def main(args=None):
    """
    Main function to initialize the ROS 2 client library, create the service server,
    and spin to process callbacks until the node is shut down.
    """
    # Initialize the ROS 2 client library
    rclpy.init(args=args)

    # Create an instance of the TimeServiceServer class
    time_service_server = TimeServiceServer()

    # Keep the node running until it is shut down
    try:
        rclpy.spin(time_service_server)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up and shut down the node
        time_service_server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()