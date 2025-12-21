#!/usr/bin/env python3
"""
Full AI Agent Controlling Humanoid Example for ROS 2 Fundamentals Module

This node demonstrates a complete integrated system with an AI agent that:
1. Receives sensor data from a humanoid robot
2. Processes the data using AI decision-making logic
3. Sends commands to control the humanoid robot
4. Uses URDF model information for proper control
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import String, Float64MultiArray
from builtin_interfaces.msg import Duration
import math
import random
from collections import deque


class AIHumanoidController(Node):
    """
    A complete AI agent that integrates perception, decision making,
    and action to control a humanoid robot using ROS 2.
    """

    def __init__(self):
        # Initialize the node with the name 'ai_humanoid_controller'
        super().__init__('ai_humanoid_controller')
        
        # Publishers for sending commands to the robot
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.joint_command_publisher = self.create_publisher(Float64MultiArray, '/joint_commands', 10)
        self.ai_status_publisher = self.create_publisher(String, '/ai_agent_status', 10)
        
        # Subscriptions for receiving data from the robot
        self.scan_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
        
        self.image_subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
        
        # Timer for the perception-action loop
        self.timer = self.create_timer(0.1, self.perception_action_loop)  # 10Hz loop
        
        # Store sensor and state data
        self.latest_scan = None
        self.latest_image = None
        self.scan_buffer = deque(maxlen=5)  # Buffer for temporal analysis
        self.state = "IDLE"
        self.target_location = None
        self.current_location = Point(x=0.0, y=0.0, z=0.0)
        
        self.get_logger().info('AI Humanoid Controller node initialized')

    def scan_callback(self, msg):
        """
        Callback function for handling laser scan data from the robot.
        """
        self.latest_scan = msg
        self.scan_buffer.append(msg)
        self.get_logger().info(f'Received laser scan with {len(msg.ranges)} readings')

    def image_callback(self, msg):
        """
        Callback function for handling camera image data from the robot.
        In a real implementation, this would process the image using computer vision.
        """
        self.latest_image = msg
        self.get_logger().info(f'Received image: {msg.width}x{msg.height}')

    def perception_action_loop(self):
        """
        Main implementation of the perception-action loop that integrates
        all the concepts from the module: ROS 2 communication, AI decision making,
        and robot control based on URDF model.
        """
        # 1. PERCEPTION: Process sensor data
        environment_state = self.perceive_environment()
        
        # 2. PROCESSING: Run AI algorithm with sensor data
        ai_decision = self.ai_decision_making(environment_state)
        
        # 3. PLANNING: Plan actions based on AI decision
        action_plan = self.plan_action(ai_decision, environment_state)
        
        # 4. ACTION: Execute the plan by sending commands to the robot
        self.execute_action(action_plan)
        
        # 5. UPDATE STATE: Update the AI agent's internal state
        self.update_state(action_plan)
        
        # Publish status for monitoring
        self.publish_status()

    def perceive_environment(self):
        """
        Process sensor data to understand the current environment.
        This integrates the ROS 2 communication patterns learned in Chapter 1.
        """
        state = {
            'obstacles': [],
            'clear_path': True,
            'object_detected': False,
            'front_clear_distance': float('inf')
        }
        
        if self.latest_scan is not None:
            # Analyze laser scan for obstacles
            ranges = self.latest_scan.ranges
            front_ranges = ranges[:len(ranges)//6] + ranges[-len(ranges)//6:]  # Front 60 degrees
            
            # Find closest obstacle in front
            valid_ranges = [r for r in front_ranges if not math.isnan(r) and r > 0]
            if valid_ranges:
                state['front_clear_distance'] = min(valid_ranges)
                state['clear_path'] = state['front_clear_distance'] > 0.8  # 0.8m threshold
                
                # Store obstacle positions relative to robot
                angle_increment = self.latest_scan.angle_increment
                for i, r in enumerate(front_ranges):
                    if not math.isnan(r) and r < 2.0:  # Obstacles within 2m
                        angle = self.latest_scan.angle_min + i * angle_increment
                        x = r * math.cos(angle)
                        y = r * math.sin(angle)
                        state['obstacles'].append({'x': x, 'y': y, 'distance': r})
        
        # In a real system, we would also process camera data here
        if self.latest_image is not None:
            # Process camera image for object detection
            # This would typically use computer vision libraries
            pass
        
        return state

    def ai_decision_making(self, environment_state):
        """
        AI decision making process based on environmental state.
        This represents the AI agent connection to ROS 2 controllers from Chapter 2.
        """
        decision = {
            'behavior': 'EXPLORE',  # Default behavior
            'target_velocity': Twist(),
            'priority': 'NORMAL'
        }
        
        # High-level decision logic based on environment
        if not environment_state['clear_path']:
            # Obstacle detected, need to avoid
            decision['behavior'] = 'AVOID_OBSTACLE'
            decision['priority'] = 'HIGH'
            
            # Calculate avoidance direction based on obstacle positions
            if environment_state['obstacles']:
                # Simple strategy: turn away from the closest obstacle
                closest_obstacle = min(environment_state['obstacles'], key=lambda o: o['distance'])
                if closest_obstacle['y'] > 0:  # Obstacle on the right
                    decision['target_velocity'].angular.z = 0.5  # Turn left
                    decision['target_velocity'].linear.x = 0.0
                else:  # Obstacle on the left
                    decision['target_velocity'].angular.z = -0.5  # Turn right
                    decision['target_velocity'].linear.x = 0.0
        else:
            # Path is clear, continue with exploration
            decision['target_velocity'].linear.x = 0.3  # Move forward
            decision['target_velocity'].angular.z = random.uniform(-0.2, 0.2)  # Small random turn
        
        # Behavioral decision based on state
        if self.state == "EXPLORING":
            decision['behavior'] = 'EXPLORE'
        elif self.state == "AVOIDING":
            decision['behavior'] = 'AVOID_OBSTACLE'
        elif self.state == "REACHED_TARGET":
            decision['behavior'] = 'WAIT'
        
        return decision

    def plan_action(self, ai_decision, environment_state):
        """
        Plan specific robot commands based on AI decision.
        This considers the URDF model structure from Chapter 3.
        """
        plan = {
            'velocity_command': Twist(),
            'joint_commands': Float64MultiArray(),
            'behavior': ai_decision['behavior'],
            'priority': ai_decision['priority']
        }
        
        # Set velocity command based on AI decision
        plan['velocity_command'] = ai_decision['target_velocity']
        
        # In a full humanoid implementation, we would also plan joint movements
        # based on the URDF model. For simplicity, we'll send a placeholder.
        plan['joint_commands'].data = [0.0] * 10  # 10 joint positions (placeholder)
        
        # Simulate humanoid-specific planning based on URDF concepts
        # In a real implementation, this would consider the robot's kinematic chain
        if ai_decision['behavior'] == 'AVOID_OBSTACLE':
            # Adjust joint positions for stability when turning
            plan['joint_commands'].data[0] = 0.1  # Adjust hip position for balance
        elif ai_decision['behavior'] == 'EXPLORE':
            # Adjust joint positions for efficient walking
            plan['joint_commands'].data[0] = 0.0  # Neutral position
        
        return plan

    def execute_action(self, action_plan):
        """
        Execute the planned actions by sending commands to the robot.
        This uses ROS 2 communication patterns to control the robot.
        """
        # Publish velocity command to the robot
        self.cmd_vel_publisher.publish(action_plan['velocity_command'])
        
        # Publish joint commands to the robot
        self.joint_command_publisher.publish(action_plan['joint_commands'])
        
        # Log the action for debugging
        self.get_logger().info(
            f"Executing {action_plan['behavior']} action: "
            f"linear.x={action_plan['velocity_command'].linear.x}, "
            f"angular.z={action_plan['velocity_command'].angular.z}")

    def update_state(self, action_plan):
        """
        Update the AI agent's internal state based on the action taken.
        """
        if action_plan['behavior'] == 'AVOID_OBSTACLE':
            self.state = "AVOIDING"
        elif action_plan['behavior'] == 'EXPLORE':
            self.state = "EXPLORING"
        elif action_plan['behavior'] == 'WAIT':
            self.state = "REACHED_TARGET"

    def publish_status(self):
        """
        Publish AI agent status for monitoring and debugging.
        """
        status_msg = String()
        status_msg.data = f"State: {self.state}, Behavior: {self.ai_decision_making(self.perceive_environment())['behavior']}"
        self.ai_status_publisher.publish(status_msg)


def main(args=None):
    """
    Main function to initialize the ROS 2 client library, create the AI humanoid controller,
    and spin to process callbacks until the node is shut down.
    """
    # Initialize the ROS 2 client library
    rclpy.init(args=args)

    # Create an instance of the AIHumanoidController class
    ai_humanoid_controller = AIHumanoidController()

    # Keep the node running until it is shut down
    try:
        rclpy.spin(ai_humanoid_controller)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up and shut down the node
        ai_humanoid_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()