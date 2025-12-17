---
title: "Lab Exercise 4: Implementing Safety Validation Layers"
sidebar_position: 2
---

# Lab Exercise 4: Implementing Safety Validation Layers

## Objectives

In this lab exercise, you will:

1. Implement a comprehensive safety validation system for the VLA pipeline
2. Create validation layers for semantic, contextual, and physical safety
3. Integrate emergency stop functionality
4. Test the safety system with potentially dangerous scenarios
5. Implement multimodal verification for action validation

## Prerequisites

Before starting this lab, ensure you have:

- A working ROS 2 Humble Hawksbill installation
- Python 3.8 or higher
- Completed all previous chapters (speech processing, LLM planning, vision-action integration)
- Basic understanding of ROS 2 concepts (nodes, topics, services, parameters)

## Estimated Time

This lab should take approximately 75-90 minutes to complete.

## Setup Instructions

1. Create a new ROS 2 package for this lab:
   ```bash
   cd ~/ros2_ws/src
   ros2 pkg create --build-type ament_python safety_validation_lab --dependencies rclpy std_msgs builtin_interfaces
   cd safety_validation_lab
   ```

2. Create the following directory structure:
   ```
   safety_validation_lab/
   ├── safety_validation_lab/
   │   ├── __init__.py
   │   ├── safety_validator_node.py
   │   ├── emergency_stop_node.py
   │   └── multimodal_verifier_node.py
   ├── test/
   │   ├── __init__.py
   │   └── test_safety_validators.py
   ├── setup.cfg
   ├── setup.py
   └── package.xml
   ```

## Procedure

### Step 1: Define Helper Classes

First, create the validation classes that will be used in the safety system:

```python
# safety_validation_lab/safety_validation_lab/validation_classes.py
import re

class SemanticValidator:
    def __init__(self):
        # Define unsafe command patterns
        self.unsafe_patterns = [
            r'.*(harm|injure|wound|injury|attack|destroy|break|damage|kill|hurt).*',
            r'.*(explosive|weapon|gun|fire|burn|ignite).*',
            r'.*emergency.*stop.*now.*',  # Emergency commands in the wrong context
        ]
    
    def validate_command(self, command):
        """Validate a command for potential unsafe content"""
        if not isinstance(command, str):
            return False, "Command must be a string"
        
        for pattern in self.unsafe_patterns:
            if re.search(pattern, command.lower()):
                return False, f"Command contains potentially unsafe pattern: {pattern}"
        
        return True, "Command appears safe"

class ContextualValidator:
    def __init__(self, safety_constraints=None):
        self.safety_constraints = safety_constraints or [
            {
                'type': 'forbidden_zone',
                'name': 'human_workspace',
                'boundary': {
                    'x_range': [-0.5, 0.5],
                    'y_range': [0.0, 1.0], 
                    'z_range': [0.0, 1.5]
                }
            },
            {
                'type': 'speed_limit',
                'value': 0.5,  # m/s
                'area': 'near_human'
            }
        ]
    
    def validate_action_in_context(self, action, robot_state=None, environment=None):
        """Validate an action in the current context"""
        robot_state = robot_state or {}
        environment = environment or {}
        
        if action['type'] == 'move_to':
            # Check if destination is in a safety-restricted area
            x = action['parameters'].get('x', 0)
            y = action['parameters'].get('y', 0)
            z = action['parameters'].get('z', 0)
            
            for constraint in self.safety_constraints:
                if (constraint['type'] == 'forbidden_zone' and
                    self.is_in_zone((x, y, z), constraint['boundary'])):
                    return False, f"Action would move robot into forbidden zone: {constraint['name']}"
        
        elif action['type'] == 'grab_object':
            # Check if the object is safe to grab
            obj_name = action['parameters'].get('object_name', '')
            if environment.get(obj_name, {}).get('fragile', False):
                return False, f"Object {obj_name} is marked as fragile and should be handled with care"
            elif environment.get(obj_name, {}).get('hot', False):
                return False, f"Object {obj_name} is marked as hot and should not be grabbed"
        
        return True, "Action is contextually appropriate"
    
    def is_in_zone(self, point, boundary):
        """Check if a point is within a boundary"""
        x, y, z = point
        min_x, max_x = boundary['x_range']
        min_y, max_y = boundary['y_range']
        min_z, max_z = boundary['z_range']
        
        return (min_x <= x <= max_x and 
                min_y <= y <= max_y and 
                min_z <= z <= max_z)

class PhysicalValidator:
    def __init__(self, robot_properties=None):
        self.robot_properties = robot_properties or {
            'workspace': {
                'min_x': -1.0, 'max_x': 1.0,
                'min_y': -1.0, 'max_y': 1.0,
                'min_z': 0.0, 'max_z': 1.5
            },
            'payload_limit': 2.0,  # kg
            'speed_limits': {
                'translation': 1.0,  # m/s
                'rotation': 0.5     # rad/s
            }
        }
    
    def validate_action_physical(self, action):
        """Validate an action against physical constraints"""
        if action['type'] == 'move_to':
            x = action['parameters'].get('x', 0)
            y = action['parameters'].get('y', 0)
            z = action['parameters'].get('z', 0)
            
            # Check workspace limits
            limits = self.robot_properties['workspace']
            if not (limits['min_x'] <= x <= limits['max_x'] and
                    limits['min_y'] <= y <= limits['max_y'] and
                    limits['min_z'] <= z <= limits['max_z']):
                return False, f"Position ({x}, {y}, {z}) is outside workspace limits"
        
        elif action['type'] == 'grab_object':
            obj_weight = action['parameters'].get('weight', 0)
            payload_limit = self.robot_properties['payload_limit']
            if obj_weight > payload_limit:
                return False, f"Object weight ({obj_weight}kg) exceeds payload limit ({payload_limit}kg)"
        
        return True, "Action is physically possible"
```

### Step 2: Implement the Safety Validator Node

Create the main safety validation node:

```python
# safety_validation_lab/safety_validation_lab/safety_validator_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
from validation_classes import SemanticValidator, ContextualValidator, PhysicalValidator

class SafetyValidatorNode(Node):
    def __init__(self):
        super().__init__('safety_validator_node')
        
        # Subscribe to raw action plans
        self.plan_sub = self.create_subscription(
            String,
            'action_plan',
            self.plan_callback,
            10
        )
        
        # Publish validated action plans
        self.validated_plan_pub = self.create_publisher(
            String, 
            'validated_action_plan', 
            10
        )
        
        # Publish safety validation status
        self.status_pub = self.create_publisher(
            String,
            'safety_validation_status',
            10
        )
        
        # Subscribe to environmental context
        self.context_sub = self.create_subscription(
            String,
            'environment_context',
            self.context_callback,
            10
        )
        
        # Store environmental context
        self.environment = {}
        self.robot_state = {}
        
        # Initialize validators
        self.semantic_validator = SemanticValidator()
        self.contextual_validator = ContextualValidator()
        self.physical_validator = PhysicalValidator()
        
        self.get_logger().info("Safety Validator Node initialized")
    
    def context_callback(self, msg):
        """Update environmental context"""
        try:
            context = json.loads(msg.data)
            self.environment.update(context.get('environment', {}))
            self.robot_state.update(context.get('robot_state', {}))
        except json.JSONDecodeError:
            self.get_logger().error(f"Invalid JSON in context: {msg.data}")
    
    def validate_plan(self, plan):
        """Validate an entire action plan"""
        validation_results = {
            'is_valid': True,
            'issues': [],
            'validated_plan': []
        }
        
        if not isinstance(plan, list):
            validation_results['is_valid'] = False
            validation_results['issues'].append("Plan must be a list of actions")
            return validation_results
        
        for i, action in enumerate(plan):
            # Check action structure
            if not isinstance(action, dict) or 'type' not in action or 'parameters' not in action:
                validation_results['is_valid'] = False
                validation_results['issues'].append(f"Step {i}: Invalid action structure")
                continue
            
            # Perform all validations
            semantic_ok, semantic_msg = self.semantic_validator.validate_command(
                json.dumps(action)
            )
            if not semantic_ok:
                validation_results['is_valid'] = False
                validation_results['issues'].append(f"Step {i}: Semantic validation failed - {semantic_msg}")
            
            contextual_ok, contextual_msg = self.contextual_validator.validate_action_in_context(
                action, self.robot_state, self.environment
            )
            if not contextual_ok:
                validation_results['is_valid'] = False
                validation_results['issues'].append(f"Step {i}: Contextual validation failed - {contextual_msg}")
            
            physical_ok, physical_msg = self.physical_validator.validate_action_physical(action)
            if not physical_ok:
                validation_results['is_valid'] = False
                validation_results['issues'].append(f"Step {i}: Physical validation failed - {physical_msg}")
            
            # If all validations pass, add to validated plan
            if semantic_ok and contextual_ok and physical_ok:
                validation_results['validated_plan'].append(action)
        
        return validation_results
    
    def plan_callback(self, msg):
        """Callback for incoming action plans"""
        try:
            plan = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().error(f"Invalid JSON in plan: {msg.data}")
            return
        
        # Validate the plan
        validation_results = self.validate_plan(plan)
        
        # Publish validation status
        status_msg = String()
        status_msg.data = json.dumps({
            'original_plan_size': len(plan),
            'validated_plan_size': len(validation_results['validated_plan']),
            'is_valid': validation_results['is_valid'],
            'issues': validation_results['issues']
        })
        self.status_pub.publish(status_msg)
        
        if validation_results['is_valid'] and validation_results['validated_plan']:
            # Publish the validated plan
            validated_msg = String()
            validated_msg.data = json.dumps(validation_results['validated_plan'])
            self.validated_plan_pub.publish(validated_msg)
            
            self.get_logger().info(f"Plan validated successfully: {len(validation_results['validated_plan'])} actions")
        else:
            self.get_logger().warn(f"Plan validation failed: {validation_results['issues']}")

def main(args=None):
    rclpy.init(args=args)
    
    validator_node = SafetyValidatorNode()
    
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

### Step 3: Implement Emergency Stop Node

Create a node that handles emergency stops:

```python
# safety_validation_lab/safety_validation_lab/emergency_stop_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

class EmergencyStopNode(Node):
    def __init__(self):
        super().__init__('emergency_stop_node')
        
        # Subscriber for emergency stop commands
        self.emergency_command_sub = self.create_subscription(
            String,
            'emergency_stop_command',
            self.emergency_stop_command_callback,
            10
        )
        
        # Subscriber for robot status (to detect anomalies)
        self.robot_status_sub = self.create_subscription(
            String,
            'robot_status',
            self.status_callback,
            10
        )
        
        # Publisher to stop all robot actions
        self.stop_command_pub = self.create_publisher(String, 'robot_stop_command', 10)
        
        # Publisher for emergency status
        self.emergency_status_pub = self.create_publisher(String, 'emergency_status', 10)
        
        # Set of emergency stop triggers
        self.emergency_triggers = {'emergency', 'stop', 'help', 'danger', 'halt', 'cease'}
        
        self.is_emergency_active = False
        
        self.get_logger().info("Emergency Stop Node initialized")
    
    def emergency_stop_command_callback(self, msg):
        """Handle emergency stop commands from various sources"""
        command = msg.data.lower()
        
        if any(trigger in command for trigger in self.emergency_triggers):
            self.trigger_emergency_stop(f"Emergency triggered by command: {msg.data}")
    
    def status_callback(self, msg):
        """Monitor robot status for dangerous conditions"""
        try:
            status = json.loads(msg.data)
            
            # Check for dangerous conditions
            if status.get('collision_detected', False):
                self.trigger_emergency_stop(f"Collision detected: {status}")
            elif status.get('temperature', 0) > 80:  # Overheating
                self.trigger_emergency_stop(f"Robot overheating: {status['temperature']}°C")
            elif status.get('current', 0) > 10:     # Overcurrent
                self.trigger_emergency_stop(f"Motor overcurrent: {status['current']}A")
            elif status.get('position_error', 0) > 0.2:  # Excessive position error
                self.trigger_emergency_stop(f"Excessive position error: {status['position_error']}m")
        
        except json.JSONDecodeError:
            self.get_logger().error(f"Invalid JSON in status: {msg.data}")
    
    def trigger_emergency_stop(self, reason):
        """Trigger emergency stop sequence"""
        if self.is_emergency_active:
            self.get_logger().warn("Emergency stop already active")
            return
        
        self.get_logger().error(f"EMERGENCY STOP ACTIVATED! Reason: {reason}")
        self.is_emergency_active = True
        
        # Publish stop command
        stop_msg = String()
        stop_msg.data = json.dumps({
            "command": "EMERGENCY_STOP",
            "reason": reason,
            "timestamp": self.get_clock().now().to_msg().sec
        })
        self.stop_command_pub.publish(stop_msg)
        
        # Publish emergency status
        status_msg = String()
        status_msg.data = json.dumps({
            "emergency_active": True,
            "reason": reason,
            "timestamp": self.get_clock().now().to_msg().sec
        })
        self.emergency_status_pub.publish(status_msg)
        
        # Additional safety measures could be implemented here:
        # - Cut power to actuators
        # - Activate audible/visual alarms
        # - Log the incident
        # - Notify human operators

def main(args=None):
    rclpy.init(args=args)
    
    emergency_node = EmergencyStopNode()
    
    try:
        rclpy.spin(emergency_node)
    except KeyboardInterrupt:
        pass
    finally:
        emergency_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Step 4: Implement Multimodal Verification Node

Create a node that performs multimodal verification:

```python
# safety_validation_lab/safety_validation_lab/multimodal_verifier_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

class MultimodalVerifierNode(Node):
    def __init__(self):
        super().__init__('multimodal_verifier_node')
        
        # Subscribe to action plans that need verification
        self.plan_sub = self.create_subscription(
            String,
            'pre_verification_plan',
            self.plan_callback,
            10
        )
        
        # Subscribe to vision data
        self.vision_sub = self.create_subscription(
            String,
            'detected_objects',
            self.vision_callback,
            10
        )
        
        # Subscribe to language input
        self.language_sub = self.create_subscription(
            String,
            'transcribed_text',
            self.language_callback,
            10
        )
        
        # Subscribe to robot state
        self.state_sub = self.create_subscription(
            String,
            'robot_state',
            self.state_callback,
            10
        )
        
        # Publish verified action plans
        self.verified_plan_pub = self.create_publisher(
            String, 
            'verified_action_plan', 
            10
        )
        
        # Store multimodal data
        self.vision_data = {}
        self.language_input = ""
        self.robot_state = {}
        
        self.get_logger().info("Multimodal Verifier Node initialized")
    
    def vision_callback(self, msg):
        """Update visual information"""
        try:
            data = json.loads(msg.data)
            self.vision_data = {obj['label']: obj for obj in data}
        except json.JSONDecodeError:
            self.get_logger().error(f"Invalid JSON in vision data: {msg.data}")
    
    def language_callback(self, msg):
        """Update language input"""
        self.language_input = msg.data
    
    def state_callback(self, msg):
        """Update robot state"""
        try:
            self.robot_state = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().error(f"Invalid JSON in state data: {msg.data}")
    
    def verify_action(self, action, language_input, vision_data, robot_state):
        """Verify an action using multiple modalities"""
        verification_score = 0
        components = 0
        
        # Language verification
        language_ok = self.verify_language_action(action, language_input)
        if language_ok:
            verification_score += 0.8  # High weight for language match
            components += 1
        
        # Vision verification
        vision_ok = self.verify_vision_action(action, vision_data, robot_state)
        if vision_ok:
            verification_score += 0.7  # High weight for vision match
            components += 1
        
        # Additional modalities could be added here
        
        if components > 0:
            avg_score = verification_score / components
            return avg_score > 0.7  # Threshold for approval
        else:
            return False  # No verification possible
    
    def verify_language_action(self, action, language_input):
        """Verify that the action matches the intended language command"""
        action_str = json.dumps(action).lower()
        language_str = language_input.lower()
        
        # Check if action aligns with language intent
        action_type = action.get('type', '')
        
        if 'grab' in language_str and 'grab' in action_str:
            return True
        if 'move' in language_str and 'move' in action_str:
            return True
        if 'place' in language_str and 'place' in action_str:
            return True
        if 'say' in language_str and 'say' in action_str:
            return True
        
        return False
    
    def verify_vision_action(self, action, vision_data, robot_state):
        """Verify that the action is appropriate given visual observations"""
        if action['type'] == 'grab_object':
            obj_name = action['parameters'].get('object_name', '')
            
            # Check if the object exists in the visual scene
            if obj_name in vision_data:
                obj_info = vision_data[obj_name]
                if obj_info.get('confidence', 0) > 0.5:
                    return True
                else:
                    self.get_logger().warn(f"Object {obj_name} has low confidence: {obj_info.get('confidence', 0)}")
                    return False
            else:
                self.get_logger().warn(f"Object {obj_name} not visible in current scene")
                return False
        
        elif action['type'] == 'move_to' and 'object_name' in action['parameters']:
            # If we're moving toward an object, check if it's visible
            obj_name = action['parameters']['object_name']
            if obj_name in vision_data:
                return True
            else:
                self.get_logger().warn(f"Target object {obj_name} not visible for movement")
                return False
        
        # For other action types, assume they're valid if they don't require vision
        return True
    
    def plan_callback(self, msg):
        """Callback for action plans needing verification"""
        try:
            plan = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().error(f"Invalid JSON in plan: {msg.data}")
            return
        
        self.get_logger().info(f"Received plan with {len(plan)} actions for multimodal verification")
        
        # Verify each action in the plan
        verified_plan = []
        for i, action in enumerate(plan):
            is_verified = self.verify_action(action, self.language_input, self.vision_data, self.robot_state)
            
            if is_verified:
                verified_plan.append(action)
                self.get_logger().info(f"Action {i} verified successfully: {action['type']}")
            else:
                self.get_logger().warn(f"Action {i} failed verification: {action['type']}")
        
        # Publish the verified plan if it's not empty
        if verified_plan:
            verified_msg = String()
            verified_msg.data = json.dumps(verified_plan)
            self.verified_plan_pub.publish(verified_msg)
            
            self.get_logger().info(f"Published verified plan with {len(verified_plan)} actions")
        else:
            self.get_logger().warn("No actions passed verification")

def main(args=None):
    rclpy.init(args=args)
    
    verifier_node = MultimodalVerifierNode()
    
    try:
        rclpy.spin(verifier_node)
    except KeyboardInterrupt:
        pass
    finally:
        verifier_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Step 5: Create a Simple Test Script

Create a basic test script to validate the safety components:

```python
# safety_validation_lab/test/test_safety_validators.py
import unittest
from safety_validation_lab.validation_classes import SemanticValidator, ContextualValidator, PhysicalValidator

class TestSafetyValidators(unittest.TestCase):
    def setUp(self):
        self.semantic_validator = SemanticValidator()
        self.contextual_validator = ContextualValidator([
            {
                'type': 'forbidden_zone',
                'name': 'test_zone',
                'boundary': {
                    'x_range': [-0.5, 0.5],
                    'y_range': [-0.5, 0.5],
                    'z_range': [0.0, 1.0]
                }
            }
        ])
        self.physical_validator = PhysicalValidator({
            'workspace': {
                'min_x': -1.0, 'max_x': 1.0,
                'min_y': -1.0, 'max_y': 1.0,
                'min_z': 0.0, 'max_z': 1.5
            },
            'payload_limit': 2.0  # kg
        })
    
    def test_semantic_validation(self):
        # Safe command
        safe_ok, safe_msg = self.semantic_validator.validate_command("Move the object to the left")
        self.assertTrue(safe_ok)
        
        # Unsafe command
        unsafe_ok, unsafe_msg = self.semantic_validator.validate_command("Harm the person")
        self.assertFalse(unsafe_ok)
        
        # Another unsafe command
        unsafe2_ok, unsafe2_msg = self.semantic_validator.validate_command("Destroy the equipment")
        self.assertFalse(unsafe2_ok)
    
    def test_contextual_validation(self):
        # Safe action
        safe_action = {
            'type': 'move_to',
            'parameters': {'x': 0.7, 'y': 0.7, 'z': 0.5}
        }
        safe_ok, _ = self.contextual_validator.validate_action_in_context(
            safe_action, {}, {}
        )
        self.assertTrue(safe_ok)
        
        # Unsafe action (in forbidden zone)
        unsafe_action = {
            'type': 'move_to',
            'parameters': {'x': 0.0, 'y': 0.0, 'z': 0.5}
        }
        unsafe_ok, _ = self.contextual_validator.validate_action_in_context(
            unsafe_action, {}, {}
        )
        self.assertFalse(unsafe_ok)
    
    def test_physical_validation(self):
        # Valid position
        valid_action = {
            'type': 'move_to',
            'parameters': {'x': 0.5, 'y': 0.5, 'z': 1.0}
        }
        valid_ok, _ = self.physical_validator.validate_action_physical(valid_action)
        self.assertTrue(valid_ok)
        
        # Invalid position (outside workspace)
        invalid_action = {
            'type': 'move_to',
            'parameters': {'x': 2.0, 'y': 0.0, 'z': 0.0}
        }
        invalid_ok, _ = self.physical_validator.validate_action_physical(invalid_action)
        self.assertFalse(invalid_ok)
        
        # Valid grab
        valid_grab = {
            'type': 'grab_object',
            'parameters': {'weight': 1.0}
        }
        valid_grab_ok, _ = self.physical_validator.validate_action_physical(valid_grab)
        self.assertTrue(valid_grab_ok)
        
        # Invalid grab (too heavy)
        invalid_grab = {
            'type': 'grab_object',
            'parameters': {'weight': 5.0}
        }
        invalid_grab_ok, _ = self.physical_validator.validate_action_physical(invalid_grab)
        self.assertFalse(invalid_grab_ok)

def run_tests():
    """Run all tests"""
    unittest.main()

if __name__ == '__main__':
    run_tests()
```

### Step 6: Update setup.py

Update the `setup.py` file to make your nodes executable:

```python
from setuptools import find_packages, setup

package_name = 'safety_validation_lab'

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
    description='A package for safety validation in VLA robotics systems',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'safety_validator = safety_validation_lab.safety_validator_node:main',
            'emergency_stop = safety_validation_lab.emergency_stop_node:main',
            'multimodal_verifier = safety_validation_lab.multimodal_verifier_node:main',
        ],
    },
)
```

### Step 7: Build and Test the Package

1. Build your package:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select safety_validation_lab
   source install/setup.bash
   ```

2. Run the safety validation test:
   ```bash
   python3 -m pytest safety_validation_lab/test/test_safety_validators.py -v
   ```

3. To integrate with the full VLA pipeline, run the safety validator node:
   ```bash
   ros2 run safety_validation_lab safety_validator
   ```

4. To test the emergency stop functionality:
   ```bash
   ros2 run safety_validation_lab emergency_stop
   ```

5. To test multimodal verification:
   ```bash
   ros2 run safety_validation_lab multimodal_verifier
   ```

6. You can trigger an emergency stop by publishing to the emergency topic:
   ```bash
   ros2 topic pub /emergency_stop_command std_msgs/String "data: 'emergency stop now'"
   ```

## Testing Scenarios

### Scenario 1: Safe Command Validation
1. Publish a safe action plan:
   ```bash
   ros2 topic pub /action_plan std_msgs/String "data: '[{\"type\": \"move_to\", \"parameters\": {\"x\": 0.5, \"y\": 0.5, \"z\": 0.5}}]'"
   ```
2. Verify that the plan passes all validations and is published to `/validated_action_plan`

### Scenario 2: Unsafe Command Detection
1. Publish an unsafe action plan:
   ```bash
   ros2 topic pub /action_plan std_msgs/String "data: '[{\"type\": \"move_to\", \"parameters\": {\"x\": 0.0, \"y\": 0.0, \"z\": 0.5}}]'"
   ```
2. Verify that the action is detected as unsafe (position is in forbidden zone) and validation fails

### Scenario 3: Emergency Stop
1. Start the emergency stop node
2. Publish an emergency command:
   ```bash
   ros2 topic pub /emergency_stop_command std_msgs/String "data: 'emergency stop'"
   ```
3. Verify that the emergency status is published and the stop command is issued

## Expected Output

When running the complete safety validation system:

1. When a safe plan is received, it should pass validation and be published to the validated topic
2. When an unsafe plan is received, validation should fail and issues should be reported
3. When an emergency command is received, the system should activate the emergency stop sequence
4. When multimodal verification is active, actions should be validated using multiple input sources

## Troubleshooting Tips

1. **Validation Issues**: Make sure action plans follow the expected JSON format
2. **Topic Communication**: Use `ros2 topic list` to verify all required topics exist
3. **Node Communication**: Use `ros2 run` commands to start each node individually to test
4. **Test Failures**: Check that your test environment matches the expected constraints in the validators

## Extensions (Optional)

1. Implement a more sophisticated semantic validator using NLP techniques
2. Add temporal constraints to prevent dangerous action sequences
3. Implement human-in-the-loop validation for critical actions
4. Add logging and audit functionality to track safety events

## Summary

In this lab, you've implemented comprehensive safety validation for the Vision-Language-Action pipeline. You've created multiple validation layers to ensure semantic, contextual, and physical safety, implemented emergency stop functionality, and created multimodal verification mechanisms. This completes the safety infrastructure needed for safe operation of VLA robotic systems.

The safety validation system is now an integral part of the full pipeline, ensuring that robot actions are safe before execution while maintaining the natural interaction capabilities that make VLA systems valuable.