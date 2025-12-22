#!/usr/bin/env python3
"""
Simplified OpenAI Agent Integration Example for ROS 2 Fundamentals Module

This script demonstrates a simplified integration between an AI agent
(simulating OpenAI-like functionality) and ROS 2 robot controllers.
In a real implementation, this would connect to actual OpenAI APIs.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import random
import time
import math


class SimplifiedOpenAIAgent(Node):
    """
    A simplified simulation of an OpenAI agent integrated with ROS 2.
    This represents how an advanced AI system might interact with ROS 2 controllers.
    In a real implementation, this would connect to OpenAI APIs for natural language
    processing or other AI capabilities.
    """

    def __init__(self):
        # Initialize the node with the name 'simplified_openai_agent'
        super().__init__('simplified_openai_agent')
        
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
        self.timer = self.create_timer(1.0, self.ai_decision_loop)
        
        # Store sensor and state data
        self.latest_scan = None
        self.state = "PROCESSING_COMMANDS"
        self.get_logger().info('Simplified OpenAI Agent node initialized')

    def scan_callback(self, msg):
        """
        Callback function for handling laser scan data from the robot.
        """
        self.latest_scan = msg
        self.get_logger().info(f'Received scan data: {len(msg.ranges)} readings')

    def ai_decision_loop(self):
        """
        Simulated AI decision-making process that would normally involve
        processing with an AI model (simulating OpenAI functionality).
        """
        # Simulate processing of natural language command or sensor data
        # In a real OpenAI integration, this might involve API calls to process commands
        self.get_logger().info('OpenAI Agent: Processing command/sensor data')
        
        # Publish current status
        status_msg = String()
        status_msg.data = f"State: {self.state}, Processing AI decisions"
        self.ai_status_publisher.publish(status_msg)
        
        # Simulate AI "thinking" process
        command = self.simulate_ai_processing()
        
        # Convert AI decision to robot command
        cmd_msg = self.ai_command_to_robot_command(command)
        
        # Publish the command to the robot
        self.cmd_vel_publisher.publish(cmd_msg)
        self.get_logger().info(f"AI Decision: {command}, Robot Command: linear.x={cmd_msg.linear.x}, angular.z={cmd_msg.angular.z}")

    def simulate_ai_processing(self):
        """
        Simulate the AI processing that would normally happen with OpenAI APIs.
        In reality, this would involve natural language understanding,
        planning, or other AI capabilities.
        """
        # This simulates how an AI agent might make decisions based on context
        if self.latest_scan is not None:
            # Analyze sensor data to make a decision
            closest_obstacle = min([r for r in self.latest_scan.ranges if not math.isnan(r)], default=float('inf'))
            
            if closest_obstacle < 1.0:
                # Obstacle detected, decide to turn
                return "TURN_RIGHT"
            elif closest_obstacle > 3.0:
                # Clear path, decide to move forward
                return "MOVE_FORWARD"
            else:
                # Intermediate distance, make random decision or continue current
                return random.choice(["MOVE_FORWARD", "TURN_LEFT", "TURN_RIGHT", "STOP"])
        else:
            # If no sensor data, continue current behavior
            return "CONTINUE_CURRENT"

    def ai_command_to_robot_command(self, ai_command):
        """
        Convert a high-level AI command to a specific robot command (Twist message).
        This simulates how an AI agent would translate its decisions to robot actions.
        """
        cmd_msg = Twist()
        
        if ai_command == "MOVE_FORWARD":
            cmd_msg.linear.x = 0.5  # Move forward at 0.5 m/s
            cmd_msg.angular.z = 0.0  # No rotation
        elif ai_command == "TURN_LEFT":
            cmd_msg.linear.x = 0.0   # No forward movement
            cmd_msg.angular.z = 0.3  # Turn left at 0.3 rad/s
        elif ai_command == "TURN_RIGHT":
            cmd_msg.linear.x = 0.0   # No forward movement
            cmd_msg.angular.z = -0.3 # Turn right at 0.3 rad/s
        elif ai_command == "STOP":
            cmd_msg.linear.x = 0.0   # Stop
            cmd_msg.angular.z = 0.0  # Stop
        else:  # CONTINUE_CURRENT or default
            # In a real system, we'd maintain the current command or behavior
            # For this example, continue moving forward
            cmd_msg.linear.x = 0.2
            cmd_msg.angular.z = 0.0
        
        return cmd_msg


def main(args=None):
    """
    Main function to initialize the ROS 2 client library, create the OpenAI agent,
    and spin to process callbacks until the node is shut down.
    """
    # Initialize the ROS 2 client library
    rclpy.init(args=args)

    # Create an instance of the SimplifiedOpenAIAgent class
    openai_agent = SimplifiedOpenAIAgent()

    # Keep the node running until it is shut down
    try:
        rclpy.spin(openai_agent)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up and shut down the node
        openai_agent.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()