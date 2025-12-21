#!/usr/bin/env python3
"""
AI Agent Subscriber Example for ROS 2 Fundamentals Module

This node simulates an AI agent that subscribes to sensor data from a robot.
It demonstrates how an AI system can use rclpy to receive information from robot sensors.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import math


class AIAgentSubscriber(Node):
    """
    A simulated AI agent that subscribes to sensor data from a robot.
    This represents how an AI system processes sensor information for decision making.
    """

    def __init__(self):
        # Initialize the node with the name 'ai_agent_subscriber'
        super().__init__('ai_agent_subscriber')
        
        # Create a subscription to receive laser scan data (simulating sensor input)
        self.scan_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
        
        # Create a subscription to receive robot odometry (simulating position data)
        self.odom_subscription = self.create_subscription(
            String,  # Using String as a simple example; in practice, this would be nav_msgs/Odometry
            '/robot_position',
            self.odom_callback,
            10)
        
        # Create a publisher to send commands based on sensor data
        self.cmd_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Store sensor data
        self.latest_scan = None
        self.latest_odom = None
        
        # Log that the subscriber has started
        self.get_logger().info('AI Agent Subscriber node initialized')

    def scan_callback(self, msg):
        """
        Callback function for laser scan messages.
        Processes sensor data and makes decisions based on the environment.
        """
        self.latest_scan = msg
        self.get_logger().info(f'Received laser scan with {len(msg.ranges)} readings')
        
        # Process the scan data to make decisions
        # This is a simplified example - a real AI agent would have more complex logic
        self.process_sensor_data()
        
    def odom_callback(self, msg):
        """
        Callback function for odometry messages.
        Updates the robot's position in the AI agent's understanding.
        """
        self.latest_odom = msg
        self.get_logger().info(f'Received position update: {msg.data}')

    def process_sensor_data(self):
        """
        Process the sensor data to make decisions.
        This simulates the AI decision-making process based on sensor inputs.
        """
        if self.latest_scan is None:
            return
        
        # Find the closest obstacle in the scan data
        min_distance = float('inf')
        for distance in self.latest_scan.ranges:
            if not math.isnan(distance) and distance < min_distance:
                min_distance = distance
        
        # Make a decision based on the sensor data
        # For example, if there's an obstacle closer than 1 meter, stop the robot
        cmd_msg = Twist()
        
        if min_distance < 1.0:  # Obstacle within 1 meter
            cmd_msg.linear.x = 0.0   # Stop moving forward
            cmd_msg.angular.z = 0.5  # Turn right to avoid obstacle
            self.get_logger().info(f'Obstacle detected at {min_distance:.2f}m, turning right')
        else:
            cmd_msg.linear.x = 0.5   # Move forward at 0.5 m/s
            cmd_msg.angular.z = 0.0  # Don't turn
            self.get_logger().info(f'Clear path, moving forward (closest obstacle: {min_distance:.2f}m)')
        
        # Publish the command based on sensor processing
        self.cmd_publisher.publish(cmd_msg)
        self.get_logger().info(f'Published command: linear.x={cmd_msg.linear.x}, angular.z={cmd_msg.angular.z}')


def main(args=None):
    """
    Main function to initialize the ROS 2 client library, create the AI agent subscriber,
    and spin to process callbacks until the node is shut down.
    """
    # Initialize the ROS 2 client library
    rclpy.init(args=args)

    # Create an instance of the AIAgentSubscriber class
    ai_agent_subscriber = AIAgentSubscriber()

    # Keep the node running until it is shut down
    try:
        rclpy.spin(ai_agent_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up and shut down the node
        ai_agent_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()