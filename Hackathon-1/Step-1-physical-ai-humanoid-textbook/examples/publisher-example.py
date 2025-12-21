#!/usr/bin/env python3
"""
Simple Publisher Example for ROS 2 Fundamentals Module

This node publishes messages to a topic called 'chatter' at a rate of 2 Hz.
It demonstrates the fundamental concept of a publisher in the ROS 2
publish-subscribe communication pattern.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MinimalPublisher(Node):
    """
    A simple ROS 2 publisher node that sends messages to a topic.
    """

    def __init__(self):
        # Initialize the node with the name 'minimal_publisher'
        super().__init__('minimal_publisher')
        
        # Create a publisher that will publish String messages to the 'chatter' topic
        # The QoS profile is set to 10, which means the publisher will keep up to 10
        # messages in its queue if subscribers haven't received them yet
        self.publisher_ = self.create_publisher(String, 'chatter', 10)
        
        # Create a timer that will call the timer_callback method every 0.5 seconds (2 Hz)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # Counter to keep track of the message number
        self.i = 0
        
        # Log that the publisher has started
        self.get_logger().info('Minimal publisher node initialized')

    def timer_callback(self):
        """
        This method is called every time the timer expires.
        It creates and publishes a message to the 'chatter' topic.
        """
        # Create a new String message
        msg = String()
        
        # Set the message data to include the count
        msg.data = f'Hello World: {self.i}'
        
        # Publish the message to the 'chatter' topic
        self.publisher_.publish(msg)
        
        # Log that the message was published
        self.get_logger().info(f'Publishing: "{msg.data}"')
        
        # Increment the counter
        self.i += 1


def main(args=None):
    """
    Main function to initialize the ROS 2 client library, create the publisher node,
    and spin to process callbacks until the node is shut down.
    """
    # Initialize the ROS 2 client library
    rclpy.init(args=args)

    # Create an instance of the MinimalPublisher class
    minimal_publisher = MinimalPublisher()

    # Keep the node running until it is shut down
    try:
        rclpy.spin(minimal_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up and shut down the node
        minimal_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()