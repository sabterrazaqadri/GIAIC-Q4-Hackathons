#!/usr/bin/env python3
"""
Simple Subscriber Example for ROS 2 Fundamentals Module

This node subscribes to messages from a topic called 'chatter'.
It demonstrates the fundamental concept of a subscriber in the ROS 2
publish-subscribe communication pattern.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MinimalSubscriber(Node):
    """
    A simple ROS 2 subscriber node that receives messages from a topic.
    """

    def __init__(self):
        # Initialize the node with the name 'minimal_subscriber'
        super().__init__('minimal_subscriber')
        
        # Create a subscription to the 'chatter' topic with a callback function
        # The QoS profile is set to 10, which means the subscriber will keep up to 10
        # messages in its queue if the callback hasn't processed them yet
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10)
        
        # Set the subscription to be owned by this node
        self.subscription  # prevent unused variable warning
        
        # Log that the subscriber has started
        self.get_logger().info('Minimal subscriber node initialized')

    def listener_callback(self, msg):
        """
        This method is called whenever a message is received on the 'chatter' topic.
        It logs the received message to the console.
        """
        # Log the received message
        self.get_logger().info(f'I heard: "{msg.data}"')


def main(args=None):
    """
    Main function to initialize the ROS 2 client library, create the subscriber node,
    and spin to process callbacks until the node is shut down.
    """
    # Initialize the ROS 2 client library
    rclpy.init(args=args)

    # Create an instance of the MinimalSubscriber class
    minimal_subscriber = MinimalSubscriber()

    # Keep the node running until it is shut down
    try:
        rclpy.spin(minimal_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up and shut down the node
        minimal_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()