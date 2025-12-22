---
title: "Lab Exercise 2: Creating LLM-Driven Planning Pipeline"
sidebar_position: 2
---

# Lab Exercise 2: Creating LLM-Driven Planning Pipeline

## Objectives

In this lab exercise, you will:

1. Create a ROS 2 node that uses an LLM to generate robot action sequences
2. Implement plan validation to ensure safety and feasibility
3. Integrate the LLM planner with your speech processing system from Chapter 1
4. Test the system with various natural language commands

## Prerequisites

Before starting this lab, ensure you have:

- Completed Chapter 1's speech processing implementation
- A working ROS 2 Humble Hawksbill installation
- Python 3.8 or higher
- An OpenAI API key
- Basic understanding of ROS 2 concepts (nodes, topics, parameters)

## Estimated Time

This lab should take approximately 60-75 minutes to complete.

## Setup Instructions

1. Create a new ROS 2 package for this lab:
   ```bash
   cd ~/ros2_ws/src
   ros2 pkg create --build-type ament_python llm_planning_lab --dependencies rclpy std_msgs builtin_interfaces
   cd llm_planning_lab
   ```

2. Create the following directory structure:
   ```
   llm_planning_lab/
   ├── llm_planning_lab/
   │   ├── __init__.py
   │   ├── llm_planner_node.py
   │   ├── plan_validator_node.py
   │   └── robot_action.py
   ├── test/
   ├── setup.cfg
   ├── setup.py
   └── package.xml
   ```

3. Install additional Python dependencies:
   ```bash
   pip install openai
   ```

4. Set your OpenAI API key as an environment variable:
   ```bash
   export OPENAI_API_KEY="your-api-key-here"
   ```

## Procedure

### Step 1: Define Robot Action Classes

First, create the `robot_action.py` file to define the types of actions your robot can perform:

```python
# llm_planning_lab/llm_planning_lab/robot_action.py
from enum import Enum

class ActionType(Enum):
    MOVE_TO = "move_to"
    GRAB_OBJECT = "grab_object"
    PLACE_OBJECT = "place_object"
    OPEN_GRIPPER = "open_gripper"
    CLOSE_GRIPPER = "close_gripper"
    SAY_TEXT = "say_text"
    STOP = "stop"

class RobotAction:
    def __init__(self, action_type, parameters=None):
        self.action_type = action_type
        self.parameters = parameters or {}
    
    def to_dict(self):
        return {
            "type": self.action_type.value,
            "parameters": self.parameters
        }
    
    @classmethod
    def from_dict(cls, data):
        return cls(ActionType(data["type"]), data.get("parameters", {}))
```

### Step 2: Implement the LLM Planner Node

Create the main planning node that uses an LLM to generate robot action sequences:

```python
# llm_planning_lab/llm_planning_lab/llm_planner_node.py
import rclpy
from rclpy.node import Node
import openai
import json
from std_msgs.msg import String
from robot_action import RobotAction, ActionType

class LLMPlannerNode(Node):
    def __init__(self):
        super().__init__('llm_planner_node')
        
        # Set your OpenAI API key
        openai.api_key = self.get_parameter_or('openai_api_key', 
                                               default_value='').value
        if not openai.api_key:
            self.get_logger().warn("No OpenAI API key provided. Set it using: ros2 param set llm_planner_node openai_api_key YOUR_KEY")
        
        # Subscriber for commands (from speech processing or direct input)
        self.command_sub = self.create_subscription(
            String,
            'robot_command',
            self.command_callback,
            10
        )
        
        # Publisher for generated action plans
        self.plan_pub = self.create_publisher(String, 'action_plan', 10)
        
        self.get_logger().info("LLM Planner Node initialized")
    
    def generate_plan(self, command, robot_state=None, environment=None):
        """Generate a plan using the LLM based on the command and context"""
        # Define the robot capabilities
        capabilities = [
            "move_to(x, y, z)", 
            "grab_object(object_name)",
            "place_object(object_name, location)",
            "open_gripper()",
            "close_gripper()",
            "say_text(text)"
        ]
        
        # Prepare the prompt for the LLM
        prompt = f"""
        You are a robot task planner. Your job is to take high-level human commands and convert them into a sequence of specific robot actions.
        
        Available robot capabilities:
        - {capabilities[0]}: Move the robot's end-effector to coordinates (x, y, z)
        - {capabilities[1]}: Grab an object by name
        - {capabilities[2]}: Place an object at a location
        - {capabilities[3]}: Open the robot's gripper
        - {capabilities[4]}: Close the robot's gripper
        - {capabilities[5]}: Make the robot speak text
        
        Current robot state: {robot_state or 'Unknown'}
        Current environment: {environment or 'Unknown'}
        
        Human command: "{command}"
        
        Please respond with a JSON list of actions to execute in order. Each action should be a dictionary with 'type' and 'parameters' keys.
        Use only the capabilities listed above.
        
        Example response format:
        [
            {{"type": "move_to", "parameters": {{"x": 1.0, "y": 2.0, "z": 0.5}}}},
            {{"type": "grab_object", "parameters": {{"object_name": "red_block"}}}},
            {{"type": "move_to", "parameters": {{"x": 0.5, "y": 0.5, "z": 0.7}}}},
            {{"type": "place_object", "parameters": {{"object_name": "red_block", "location": "blue_zone"}}}}
        ]
        """
        
        try:
            response = openai.ChatCompletion.create(
                model="gpt-3.5-turbo",
                messages=[{"role": "user", "content": prompt}],
                temperature=0.3,
                max_tokens=500
            )
            
            # Extract the plan from the response
            content = response.choices[0].message['content'].strip()
            
            # Extract JSON from the response (in case it includes other text)
            start_idx = content.find('[')
            end_idx = content.rfind(']') + 1
            
            if start_idx != -1 and end_idx != 0:
                json_str = content[start_idx:end_idx]
                plan = json.loads(json_str)
                
                # Validate the plan format
                for action in plan:
                    if 'type' not in action or 'parameters' not in action:
                        raise ValueError("Invalid action format in plan")
                
                return plan
            
            else:
                self.get_logger().error(f"Could not extract JSON from LLM response: {content}")
                return []
                
        except Exception as e:
            self.get_logger().error(f"Error generating plan with LLM: {e}")
            return []
    
    def command_callback(self, msg):
        """Callback for receiving commands"""
        command = msg.data
        self.get_logger().info(f"Received command: {command}")
        
        # Generate a plan based on the command
        plan = self.generate_plan(command)
        
        if plan:
            self.get_logger().info(f"Generated plan: {plan}")
            
            # Publish the plan
            plan_msg = String()
            plan_msg.data = json.dumps(plan)
            self.plan_pub.publish(plan_msg)
        else:
            self.get_logger().warn("Could not generate a plan for the command")

def main(args=None):
    rclpy.init(args=args)
    
    llm_planner_node = LLMPlannerNode()
    
    # Add parameter for OpenAI API key
    llm_planner_node.declare_parameter('openai_api_key', '')
    
    try:
        rclpy.spin(llm_planner_node)
    except KeyboardInterrupt:
        pass
    finally:
        llm_planner_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Step 3: Implement Plan Validation

Create a validation node to ensure generated plans are safe and feasible:

```python
# llm_planning_lab/llm_planning_lab/plan_validator_node.py
import rclpy
from rclpy.node import Node
import json
from std_msgs.msg import String

class PlanValidatorNode(Node):
    def __init__(self):
        super().__init__('plan_validator_node')
        
        # Subscriber for raw plans
        self.plan_sub = self.create_subscription(
            String,
            'action_plan',
            self.plan_callback,
            10
        )
        
        # Publisher for validated plans
        self.validated_plan_pub = self.create_publisher(String, 'validated_action_plan', 10)
        
        # Publisher for validation status
        self.status_pub = self.create_publisher(String, 'plan_validation_status', 10)
        
        self.get_logger().info("Plan Validator Node initialized")
    
    def validate_plan(self, plan):
        """Validate a plan for safety and feasibility"""
        validation_results = {
            "is_valid": True,
            "issues": [],
            "suggested_fixes": []
        }
        
        if not isinstance(plan, list):
            validation_results["is_valid"] = False
            validation_results["issues"].append("Plan must be a list of actions")
            return validation_results
        
        for i, action in enumerate(plan):
            if not isinstance(action, dict):
                validation_results["is_valid"] = False
                validation_results["issues"].append(f"Action at index {i} is not a dictionary")
                continue
            
            if 'type' not in action or 'parameters' not in action:
                validation_results["is_valid"] = False
                validation_results["issues"].append(f"Action at index {i} missing 'type' or 'parameters'")
                continue
            
            action_type = action['type']
            
            # Check for potentially dangerous actions
            if action_type == "move_to":
                # Check for dangerous coordinates
                x = action['parameters'].get('x', 0)
                y = action['parameters'].get('y', 0)
                z = action['parameters'].get('z', 0)
                
                # Example safety checks - these would be more sophisticated in practice
                if z < 0:  # Don't move below ground level
                    validation_results["is_valid"] = False
                    validation_results["issues"].append(f"Action at index {i}: Attempting to move below ground level (z={z})")
            
            elif action_type == "grab_object":
                obj_name = action['parameters'].get('object_name', '')
                if not obj_name:
                    validation_results["is_valid"] = False
                    validation_results["issues"].append(f"Action at index {i}: grab_object requires object_name parameter")
        
        return validation_results
    
    def plan_callback(self, msg):
        """Callback for receiving plans to validate"""
        try:
            plan = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().error(f"Invalid JSON in plan: {msg.data}")
            return
        
        # Validate the plan
        validation_results = self.validate_plan(plan)
        
        # Publish validation status
        status_msg = String()
        status_msg.data = json.dumps(validation_results, indent=2)
        self.status_pub.publish(status_msg)
        
        if validation_results["is_valid"]:
            # Publish the validated plan
            validated_msg = String()
            validated_msg.data = msg.data  # Original plan is valid
            self.validated_plan_pub.publish(validated_msg)
            
            self.get_logger().info("Plan validation passed - published validated plan")
        else:
            self.get_logger().warn(f"Plan validation failed: {validation_results['issues']}")

def main(args=None):
    rclpy.init(args=args)
    
    validator_node = PlanValidatorNode()
    
    try:
        rclpy.spin(validator_node)
    except KeyboardInterrupt:
        pass
    finally:
        validator_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Step 4: Update setup.py

Update the `setup.py` file to make your nodes executable:

```python
from setuptools import find_packages, setup

package_name = 'llm_planning_lab'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='A package for LLM-based planning in robotics',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'llm_planner = llm_planning_lab.llm_planner_node:main',
            'plan_validator = llm_planning_lab.plan_validator_node:main',
        ],
    },
)
```

### Step 5: Integrate with Speech Processing System

To connect with the speech processing system from Chapter 1, you can simply use the same topic names. The speech system publishes to `robot_command`, which is exactly what our LLM planner subscribes to. 

### Step 6: Build and Test the Package

1. Build your package:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select llm_planning_lab
   source install/setup.bash
   ```

2. Open three separate terminals and run:

   Terminal 1 (from Chapter 1):
   ```bash
   ros2 run speech_to_text_lab audio_capture
   ```

   Terminal 2 (from Chapter 1):
   ```bash
   ros2 run speech_to_text_lab whisper_node
   ```

   Terminal 3 (from Chapter 1):
   ```bash
   ros2 run speech_to_text_lab command_interpreter
   ```

   Terminal 4:
   ```bash
   ros2 run llm_planning_lab llm_planner
   ```

   Terminal 5:
   ```bash
   ros2 run llm_planning_lab plan_validator
   ```

3. Monitor the validated plans:
   ```bash
   ros2 topic echo /validated_action_plan
   ```

4. Optionally, monitor the validation status:
   ```bash
   ros2 topic echo /plan_validation_status
   ```

## Expected Output

When you speak a command that gets processed through the entire pipeline, you should see:

1. In the audio capture terminal: Audio recording messages
2. In the whisper terminal: Transcription of your speech
3. In the command interpreter terminal: Processing of the transcribed text
4. In the LLM planner terminal: Plan generation messages
5. In the validator terminal: Plan validation results
6. In the topic monitoring terminal: Validated action plans as JSON

## Troubleshooting Tips

1. **API Key Issues**: Make sure your OpenAI API key is set as an environment variable:
   ```bash
   echo $OPENAI_API_KEY
   ```
   If it's empty, set it again:
   ```bash
   export OPENAI_API_KEY="your-api-key-here"
   ```

2. **Plan Format Issues**: If the LLM is not returning properly formatted JSON, you may need to adjust your prompt to be more specific about the required format.

3. **Connection Issues**: Verify that all nodes are communicating on the correct topics using:
   ```bash
   ros2 topic list
   ros2 topic info /robot_command
   ```

4. **API Limits**: If you're getting API errors, you may have exceeded your OpenAI usage limits.

## Extensions (Optional)

1. Implement more sophisticated environmental context for the LLM
2. Add a simulation component to test plans before physical execution
3. Implement a feedback loop where the robot reports the outcome of actions back to the LLM
4. Add multimodal capabilities by integrating vision processing with LLM planning

## Summary

In this lab, you've successfully implemented an LLM-driven planning pipeline that converts natural language commands into sequences of robot actions. By combining this with the speech processing system from Chapter 1, you now have a system that can take spoken commands and generate executable robot plans. This represents a significant component of the Vision-Language-Action pipeline.

In the next chapter, we'll add vision integration to make the system more robust and contextually aware.