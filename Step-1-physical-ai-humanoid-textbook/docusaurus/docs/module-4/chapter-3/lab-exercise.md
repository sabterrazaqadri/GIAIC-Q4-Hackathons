---
title: "Lab Exercise 3: Integrating Vision Feedback into Action Loops"
sidebar_position: 2
---

# Lab Exercise 3: Integrating Vision Feedback into Action Loops

## Objectives

In this lab exercise, you will:

1. Set up a camera interface in ROS 2
2. Implement object detection using OpenCV
3. Create a visual action interpreter that combines vision data with action planning
4. Test the entire Vision-Language-Action pipeline with real visual feedback

## Prerequisites

Before starting this lab, ensure you have:

- A working ROS 2 Humble Hawksbill installation
- Python 3.8 or higher
- A camera connected to your computer
- OpenCV installed (`pip install opencv-python`)
- Completed Chapters 1 and 2 implementations
- Basic understanding of ROS 2 concepts (nodes, topics, message types)

## Estimated Time

This lab should take approximately 75-90 minutes to complete.

## Setup Instructions

1. Create a new ROS 2 package for this lab:
   ```bash
   cd ~/ros2_ws/src
   ros2 pkg create --build-type ament_python vision_action_lab --dependencies rclpy std_msgs sensor_msgs builtin_interfaces
   cd vision_action_lab
   ```

2. Create the following directory structure:
   ```
   vision_action_lab/
   ├── vision_action_lab/
   │   ├── __init__.py
   │   ├── camera_node.py
   │   ├── object_detection_node.py
   │   └── visual_action_interpreter_node.py
   ├── test/
   ├── setup.cfg
   ├── setup.py
   └── package.xml
   ```

3. Install additional Python dependencies:
   ```bash
   pip install opencv-python
   pip install opencv-python-headless  # For headless systems
   pip install cv-bridge
   ```

## Procedure

### Step 1: Create the Camera Node

First, create a node that interfaces with your camera:

```python
# vision_action_lab/vision_action_lab/camera_node.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        self.bridge = CvBridge()
        
        # Create publisher for camera images
        self.image_pub = self.create_publisher(Image, '/camera/image_raw', 10)
        
        # Initialize camera (change index as needed - usually 0 or 1)
        self.cap = cv2.VideoCapture(0)
        
        if not self.cap.isOpened():
            self.get_logger().error("Could not open camera")
            return
            
        # Set camera properties if needed
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        
        # Timer to capture images at regular intervals (~10 FPS)
        self.timer = self.create_timer(0.1, self.capture_image)
        
        self.get_logger().info("Camera node initialized")
    
    def capture_image(self):
        ret, frame = self.cap.read()
        if ret:
            # Convert OpenCV image to ROS Image message
            ros_image = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            ros_image.header.stamp = self.get_clock().now().to_msg()
            ros_image.header.frame_id = 'camera_frame'
            self.image_pub.publish(ros_image)
        else:
            self.get_logger().warn("Failed to capture image from camera")
    
    def destroy_node(self):
        # Release camera when node is destroyed
        if self.cap.isOpened():
            self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    camera_node = CameraNode()
    
    try:
        rclpy.spin(camera_node)
    except KeyboardInterrupt:
        pass
    finally:
        camera_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Step 2: Implement Object Detection Node

Create a node that performs object detection on camera images:

```python
# vision_action_lab/vision_action_lab/object_detection_node.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
import json

class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('object_detection_node')
        self.bridge = CvBridge()
        
        # Subscribe to camera feed
        self.camera_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.camera_callback,
            10
        )
        
        # Publish detected objects
        self.object_pub = self.create_publisher(String, '/detected_objects', 10)
        
        # Create a simple color-based object detector for demonstration
        # In a real application, you would use a pre-trained model
        self.get_logger().info("Object Detection Node initialized")
    
    def detect_simple_objects(self, cv_image):
        """Detect objects based on color (for demonstration purposes)"""
        # Convert BGR to HSV
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        # Define range for red color (in HSV)
        lower_red1 = np.array([0, 50, 50])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 50, 50])
        upper_red2 = np.array([180, 255, 255])
        
        # Define range for blue color
        lower_blue = np.array([100, 50, 50])
        upper_blue = np.array([130, 255, 255])
        
        # Create masks for red and blue
        mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask_red = mask_red1 + mask_red2
        mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)
        
        # Find contours for red objects
        contours_red, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours_blue, _ = cv2.findContours(mask_blue, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        detections = []
        
        # Process red objects
        for contour in contours_red:
            if cv2.contourArea(contour) > 500:  # Filter small contours
                x, y, w, h = cv2.boundingRect(contour)
                center_x = x + w // 2
                center_y = y + h // 2
                
                detection_info = {
                    'label': 'red_object',
                    'confidence': 0.8,  # Fixed for demo
                    'bbox': {'x': int(x), 'y': int(y), 'w': int(w), 'h': int(h)},
                    'center': {'x': int(center_x), 'y': int(center_y)}
                }
                detections.append(detection_info)
        
        # Process blue objects
        for contour in contours_blue:
            if cv2.contourArea(contour) > 500:  # Filter small contours
                x, y, w, h = cv2.boundingRect(contour)
                center_x = x + w // 2
                center_y = y + h // 2
                
                detection_info = {
                    'label': 'blue_object',
                    'confidence': 0.8,  # Fixed for demo
                    'bbox': {'x': int(x), 'y': int(y), 'w': int(w), 'h': int(h)},
                    'center': {'x': int(center_x), 'y': int(center_y)}
                }
                detections.append(detection_info)
        
        return detections
    
    def camera_callback(self, msg):
        # Convert ROS Image message to OpenCV format
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Error converting image: {e}")
            return
        
        # Perform object detection
        detections = self.detect_simple_objects(cv_image)
        
        # Publish detections
        if detections:  # Only publish if detections were found
            detection_msg = String()
            detection_msg.data = json.dumps(detections)
            self.object_pub.publish(detection_msg)
            self.get_logger().info(f"Published {len(detections)} detections")

def main(args=None):
    rclpy.init(args=args)
    
    detection_node = ObjectDetectionNode()
    
    try:
        rclpy.spin(detection_node)
    except KeyboardInterrupt:
        pass
    finally:
        detection_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Step 3: Create Visual Action Interpreter Node

Implement the node that combines vision data with action planning:

```python
# vision_action_lab/vision_action_lab/visual_action_interpreter_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

class VisualActionInterpreterNode(Node):
    def __init__(self):
        super().__init__('visual_action_interpreter_node')
        
        # Subscribe to object detections
        self.object_sub = self.create_subscription(
            String,
            '/detected_objects',
            self.object_callback,
            10
        )
        
        # Subscribe to planned actions
        self.plan_sub = self.create_subscription(
            String,
            '/validated_action_plan',
            self.plan_callback,
            10
        )
        
        # Publisher for final action commands
        self.action_pub = self.create_publisher(String, '/final_robot_action', 10)
        
        # Store object information
        self.objects = {}
        
        self.get_logger().info("Visual Action Interpreter Node initialized")
    
    def object_callback(self, msg):
        """Callback for object detections"""
        try:
            detections = json.loads(msg.data)
            for detection in detections:
                label = detection['label']
                bbox = detection['bbox']
                center = detection['center']
                
                # Store object information with its location
                self.objects[label] = {
                    'bbox': bbox,
                    'center': center,
                    'confidence': detection['confidence']
                }
                
            self.get_logger().info(f"Updated object registry with {len(detections)} objects")
        except json.JSONDecodeError:
            self.get_logger().error(f"Invalid JSON in object detection: {msg.data}")
    
    def plan_callback(self, msg):
        """Callback for action plans"""
        try:
            plan = json.loads(msg.data)
            self.get_logger().info(f"Received action plan with {len(plan)} steps")
            
            # If the plan involves specific objects, enrich with visual information
            enriched_plan = self.enrich_plan_with_vision(plan)
            
            # Publish the enriched plan
            action_msg = String()
            action_msg.data = json.dumps(enriched_plan)
            self.action_pub.publish(action_msg)
            
            self.get_logger().info(f"Published enriched action plan with {len(enriched_plan)} steps")
        except json.JSONDecodeError:
            self.get_logger().error(f"Invalid JSON in action plan: {msg.data}")
    
    def enrich_plan_with_vision(self, plan):
        """Add visual information to action parameters"""
        enriched_plan = []
        
        for i, action in enumerate(plan):
            # Make a copy of the action to modify
            enriched_action = action.copy()
            
            # If the action refers to an object, add its location
            if action['type'] == 'grab_object':
                obj_name = action['parameters'].get('object_name', '')
                
                # Look up the object in our stored detections
                if obj_name in self.objects:
                    # Add location information to the action
                    obj_info = self.objects[obj_name]
                    enriched_action['parameters']['location'] = obj_info['center']
                    enriched_action['parameters']['bbox'] = obj_info['bbox']
                    
                    self.get_logger().info(f"Added location for {obj_name}: {obj_info['center']}")
                else:
                    # If object not found, report with lower confidence or skip
                    self.get_logger().warn(f"Object '{obj_name}' not found in current detections")
                    # For this demo, we'll still pass through the action but note the issue
                    enriched_action['parameters']['location'] = None
                    enriched_action['parameters']['warning'] = f"Object '{obj_name}' not visible"
            
            elif action['type'] == 'move_to' and 'object_name' in action['parameters']:
                # If we're moving toward an object, get its location
                obj_name = action['parameters']['object_name']
                
                if obj_name in self.objects:
                    obj_info = self.objects[obj_name]
                    # Override the coordinates with the detected location
                    enriched_action['parameters']['x'] = obj_info['center']['x']
                    enriched_action['parameters']['y'] = obj_info['center']['y']
                    # Note: We keep the original z coordinate from the plan
                    enriched_action['parameters']['object_location'] = obj_info['center']
                
            enriched_plan.append(enriched_action)
        
        return enriched_plan

def main(args=None):
    rclpy.init(args=args)
    
    interpreter_node = VisualActionInterpreterNode()
    
    try:
        rclpy.spin(interpreter_node)
    except KeyboardInterrupt:
        pass
    finally:
        interpreter_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Step 4: Update setup.py

Update the `setup.py` file to make your nodes executable:

```python
from setuptools import find_packages, setup

package_name = 'vision_action_lab'

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
    description='A package for vision-action integration in robotics',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_node = vision_action_lab.camera_node:main',
            'object_detection = vision_action_lab.object_detection_node:main',
            'visual_interpreter = vision_action_lab.visual_action_interpreter_node:main',
        ],
    },
)
```

### Step 5: Build and Test the Package

1. Build your package:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select vision_action_lab
   source install/setup.bash
   ```

2. Open five separate terminals and run all components of the full pipeline:

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

   Terminal 4 (from Chapter 2):
   ```bash
   export OPENAI_API_KEY="your-api-key-here"
   ros2 run llm_planning_lab llm_planner
   ```

   Terminal 5 (from Chapter 2):
   ```bash
   ros2 run llm_planning_lab plan_validator
   ```

   Terminal 6 (new vision components):
   ```bash
   ros2 run vision_action_lab camera_node
   ```

   Terminal 7 (new vision components):
   ```bash
   ros2 run vision_action_lab object_detection
   ```

   Terminal 8 (new vision components):
   ```bash
   ros2 run vision_action_lab visual_interpreter
   ```

3. Monitor the final robot actions:
   ```bash
   ros2 topic echo /final_robot_action
   ```

4. Test with a simple command by publishing to the robot command topic:
   ```bash
   ros2 topic pub /robot_command std_msgs/String "data: 'Grab the red object'"
   ```

## Expected Output

When you run the complete pipeline with the vision system active:

1. Place a red or blue object in front of your camera
2. The camera node will continuously capture images
3. The object detection node will identify and locate colored objects
4. Publishing a command like "Grab the red object" will trigger:
   - Speech processing (if using audio) → Text → Command interpretation → LLM planning → Plan validation → Visual enrichment → Final action
5. The visual action interpreter will add location information to the grab action based on what it sees
6. The final action will include the coordinates of the detected red object

You should see messages indicating that objects are being detected and that the action plan is being enriched with visual information.

## Troubleshooting Tips

1. **Camera Issues**: 
   - Check that your camera is properly connected
   - Verify the camera index (try 0, 1, or 2)
   - Test camera with `v4l2-ctl --list-devices` (on Linux)

2. **Object Detection Issues**:
   - The simple color-based detector works only for red and blue objects
   - Ensure adequate lighting for color detection
   - Adjust HSV ranges if needed for your specific objects

3. **Topic Communication Issues**:
   - Use `ros2 topic list` to verify all topics exist
   - Use `ros2 topic echo` to verify messages are flowing correctly

4. **OpenCV Installation Issues**:
   - Make sure OpenCV is properly installed (`pip install opencv-python`)
   - Check that cv_bridge is installed for ROS 2

## Extensions (Optional)

1. Implement more sophisticated object detection using a pre-trained model (YOLO, MobileNet SSD)
2. Add 3D localization using depth data if you have an RGB-D camera
3. Implement object tracking to maintain object identities across frames
4. Add visual servoing to refine actions based on real-time visual feedback

## Summary

In this lab, you've successfully integrated vision processing into the Vision-Language-Action pipeline. Your system now captures visual information, detects objects in the environment, and uses this information to enrich action plans with spatial information. This completes a crucial component of the VLA pipeline, enabling robots to operate based on what they see in addition to what they're told.

In the next chapter, we'll address critical safety considerations when combining these capabilities, implementing validation layers to ensure robot actions are safe and appropriate.