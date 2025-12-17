#!/usr/bin/env python3
"""
AI Agent Publisher Example for ROS 2 Fundamentals Module

This node simulates an AI agent that publishes commands to robot controllers.
It demonstrates how an AI system can use rclpy to send commands to a robot.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import random
import time


class AIAgentPublisher(Node):
    """
    A simulated AI agent that publishes commands to control a robot.
    This represents the AI decision-making process that outputs robot commands.
    """

    def __init__(self):
        # Initialize the node with the name 'ai_agent_publisher'
        super().__init__('ai_agent_publisher')
        
        # Create a publisher to send velocity commands to the robot
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Create a publisher to send status messages
        self.status_publisher = self.create_publisher(String, '/ai_agent_status', 10)
        
        # Create a timer to run the AI decision-making process periodically
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.ai_decision_loop)
        
        # Simple state for the AI agent
        self.state = "IDLE"
        self.get_logger().info('AI Agent Publisher node initialized')

    def ai_decision_loop(self):
        """
        Simulate the AI decision-making process.
        In a real AI agent, this would contain actual AI algorithms.
        """
        # Simulate different states for the AI agent
        states = ["MOVING_FORWARD", "TURNING", "STOPPED", "IDLE"]
        self.state = random.choice(states)
        
        # Log the current state
        self.get_logger().info(f'AI State: {self.state}')
        
        # Publish status
        status_msg = String()
        status_msg.data = f'AI Agent State: {self.state}'
        self.status_publisher.publish(status_msg)
        
        # Create and publish appropriate commands based on the state
        cmd_vel_msg = Twist()
        
        if self.state == "MOVING_FORWARD":
            cmd_vel_msg.linear.x = 0.5  # Move forward at 0.5 m/s
            cmd_vel_msg.angular.z = 0.0  # No rotation
        elif self.state == "TURNING":
            cmd_vel_msg.linear.x = 0.0   # No forward movement
            cmd_vel_msg.angular.z = 0.5  # Turn at 0.5 rad/s
        elif self.state == "STOPPED":
            cmd_vel_msg.linear.x = 0.0   # Stop
            cmd_vel_msg.angular.z = 0.0  # Stop
        else:  # IDLE
            cmd_vel_msg.linear.x = 0.2   # Slow movement
            cmd_vel_msg.angular.z = 0.0  # No rotation
        
        # Publish the command
        self.cmd_vel_publisher.publish(cmd_vel_msg)
        self.get_logger().info(f'Published command: linear.x={cmd_vel_msg.linear.x}, angular.z={cmd_vel_msg.angular.z}')


def main(args=None):
    """
    Main function to initialize the ROS 2 client library, create the AI agent publisher,
    and spin to process callbacks until the node is shut down.
    """
    # Initialize the ROS 2 client library
    rclpy.init(args=args)

    # Create an instance of the AIAgentPublisher class
    ai_agent_publisher = AIAgentPublisher()

    # Keep the node running until it is shut down
    try:
        rclpy.spin(ai_agent_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up and shut down the node
        ai_agent_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()