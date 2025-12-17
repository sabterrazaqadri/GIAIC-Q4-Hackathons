#!/usr/bin/env python3
"""
AI Agent with Complete ROS 2 Integration for ROS 2 Fundamentals Module

This node demonstrates a complete AI agent that both publishes commands to
and subscribes to data from a ROS 2 robot controller, showcasing the bidirectional
communication between AI agents and robot systems.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from example_interfaces.srv import Trigger
import math
import random


class CompleteAIAgent(Node):
    """
    A complete AI agent that demonstrates both publishing commands to robot controllers
    and subscribing to sensor data from the robot.
    """

    def __init__(self):
        # Initialize the node with the name 'complete_ai_agent'
        super().__init__('complete_ai_agent')
        
        # Publishers for sending commands to the robot
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.ai_status_publisher = self.create_publisher(String, '/ai_agent_status', 10)
        
        # Subscriptions for receiving data from the robot
        self.scan_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
        
        # Timer for AI decision-making process
        self.timer = self.create_timer(0.5, self.ai_decision_loop)
        
        # Create a service client to request robot actions
        self.navigation_client = self.create_client(Trigger, 'request_navigation_action')
        
        # Store sensor and state data
        self.latest_scan = None
        self.state = "SEARCHING"
        self.target_reached = False
        
        # Wait for navigation service to be available
        while not self.navigation_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Navigation service not available, waiting again...')
        
        self.get_logger().info('Complete AI Agent node initialized')

    def scan_callback(self, msg):
        """
        Callback function for handling laser scan data from the robot.
        """
        self.latest_scan = msg
        self.get_logger().info(f'Received scan data: {len(msg.ranges)} readings')

    def ai_decision_loop(self):
        """
        Main AI decision-making loop that processes sensor data and sends commands.
        """
        # Publish current AI status
        status_msg = String()
        status_msg.data = f"State: {self.state}, Target: {'Reached' if self.target_reached else 'Not Reached'}"
        self.ai_status_publisher.publish(status_msg)
        
        # Make decisions based on the current state and sensor data
        cmd_msg = Twist()
        
        if self.state == "SEARCHING":
            # Simple exploration behavior: move forward and occasionally turn
            cmd_msg.linear.x = 0.3
            cmd_msg.angular.z = random.uniform(-0.2, 0.2)
            
            # Check if we've detected something interesting
            if self.detect_obstacle_close():
                self.state = "APPROACHING"
        
        elif self.state == "APPROACHING":
            # Approach behavior: go towards a detected target
            cmd_msg.linear.x = 0.2
            cmd_msg.angular.z = 0.0  # Go straight
            
            # If we're getting close to the target, stop and switch state
            if self.distance_to_obstacle() < 0.5:
                cmd_msg.linear.x = 0.0
                self.state = "ANALYZING"
        
        elif self.state == "ANALYZING":
            # Analyze behavior: stop and request a specific action from the robot
            cmd_msg.linear.x = 0.0
            cmd_msg.angular.z = 0.0
            
            # Request navigation service to perform a specific action
            request = Trigger.Request()
            
            # In a real system, this would do something specific
            future = self.navigation_client.call_async(request)
            
            # For this example, just switch to the next state after a while
            self.state = "SEARCHING"
        
        # Publish the command to the robot
        self.cmd_vel_publisher.publish(cmd_msg)
        self.get_logger().info(f"State: {self.state}, Command: linear.x={cmd_msg.linear.x}, angular.z={cmd_msg.angular.z}")

    def detect_obstacle_close(self):
        """
        Check if there's an obstacle close to the robot.
        """
        if self.latest_scan is None:
            return False
        
        # Check if there's an obstacle within 1.5m in the front 60 degrees
        front_ranges = self.latest_scan.ranges[:len(self.latest_scan.ranges)//12] + \
                      self.latest_scan.ranges[-len(self.latest_scan.ranges)//12:]
        
        for distance in front_ranges:
            if not math.isnan(distance) and distance < 1.5:
                return True
        return False

    def distance_to_obstacle(self):
        """
        Get the distance to the closest obstacle in front of the robot.
        """
        if self.latest_scan is None:
            return float('inf')
        
        # Check the front 60 degrees
        front_ranges = self.latest_scan.ranges[:len(self.latest_scan.ranges)//12] + \
                      self.latest_scan.ranges[-len(self.latest_scan.ranges)//12:]
        
        min_distance = float('inf')
        for distance in front_ranges:
            if not math.isnan(distance) and distance < min_distance:
                min_distance = distance
        
        return min_distance


def main(args=None):
    """
    Main function to initialize the ROS 2 client library, create the complete AI agent,
    and spin to process callbacks until the node is shut down.
    """
    # Initialize the ROS 2 client library
    rclpy.init(args=args)

    # Create an instance of the CompleteAIAgent class
    complete_ai_agent = CompleteAIAgent()

    # Keep the node running until it is shut down
    try:
        rclpy.spin(complete_ai_agent)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up and shut down the node
        complete_ai_agent.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()