# Chapter 3: URDF Fundamentals for Humanoid Robotics

## Learning Objectives

By the end of this chapter, you will be able to:
- Understand the structure and components of URDF (Unified Robot Description Format) files
- Create basic URDF files for simple robot components
- Read and interpret existing URDF files for humanoid robots
- Explain the relationship between URDF elements and physical robot components

## Introduction

Unified Robot Description Format (URDF) is an XML-based format used in ROS to describe robots. In the context of humanoid robotics, URDF provides a standardized way to define the physical structure of humanoid robots, including their links (rigid parts), joints (connections between parts), and other properties like visual appearance and collision geometry.

URDF is crucial for humanoid robotics as it allows simulation engines, controllers, and other tools to understand the physical structure of the robot. This understanding is necessary for tasks like motion planning, simulation, and visualization. In this chapter, we'll explore the fundamentals of URDF, focusing on applications in humanoid robotics.

## Main Content

### What is URDF?

URDF stands for Unified Robot Description Format. It is an XML-based format that allows you to define the physical properties of a robot in a structured way. A URDF file contains information about:

- Links: Rigid parts of the robot (e.g., torso, limbs)
- Joints: Connections between links (e.g., joints, actuators)
- Visual properties: How the robot appears in simulation
- Collision properties: How the robot interacts physically with its environment
- Inertial properties: Mass, center of mass, and inertia for physics simulation

### URDF Structure

A URDF file is structured as an XML document with a single `robot` element as the root. The robot element contains `link` and `joint` elements that define the structure of the robot.

```xml
<?xml version="1.0"?>
<robot name="humanoid_robot">
  <!-- Links are rigid parts of the robot -->
  <link name="torso">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="5"/>
      <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/>
    </inertial>
  </link>

  <!-- Joints connect links to each other -->
  <joint name="right_hip_joint" type="revolute">
    <parent link="torso"/>
    <child link="right_leg"/>
    <origin xyz="0 -0.1 -0.25" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
  </joint>
</robot>
```

### Links

Links represent rigid parts of the robot. Each link has:
- A name (unique within the robot)
- Visual properties (shape, color, etc.)
- Collision properties (for physical interaction)
- Inertial properties (for physics simulation)

#### Visual Elements in Links
The visual element defines how the link appears in simulation and visualization tools. It includes:
- Geometry (shape and size)
- Material (color and texture)

#### Collision Elements in Links
The collision element defines the physical shape of the link for collision detection. This can be different from the visual shape for performance reasons.

#### Inertial Elements in Links
The inertial element defines the mass and inertial tensor of the link, which is critical for physics simulation.

### Joints

Joints connect links together and define the allowed motion between them. Common joint types include:
- Fixed: No movement allowed (welds two links together)
- Revolute: Single-axis rotation (like a hinge)
- Continuous: Unlimited single-axis rotation
- Prismatic: Single-axis linear translation
- Floating: 6 DOF (degrees of freedom)
- Planar: 3 DOF planar motion

### URDF for Humanoid Robots

Humanoid robots have a specific structure that URDF represents with a tree-like hierarchy:
- Torso (root link)
  - Head
  - Left Arm
    - Left Forearm
      - Left Hand
  - Right Arm
    - Right Forearm
      - Right Hand
  - Left Leg
    - Left Lower Leg
      - Left Foot
  - Right Leg
    - Right Lower Leg
      - Right Foot

Each joint represents a degree of freedom in the humanoid's movement, allowing for complex locomotion and manipulation.

### Joint Types and Constraints in Humanoid Robotics

Humanoid robots have specific joint types and constraints that mirror human anatomy. Understanding these is crucial for creating realistic humanoid robot models.

#### Joint Types in Humanoid Robots

1. **Fixed Joints**: Used for permanent connections where no movement is allowed (e.g., connecting sensors to links)
   - In URDF: `<joint type="fixed">`
   - Example: Mounting a camera to a head link

2. **Revolute Joints**: Allow rotation around a single axis, with defined limits
   - In URDF: `<joint type="revolute">`
   - Example: Elbow (flexion/extension), knee (flexion/extension)
   - Limits are crucial for humanoid joints to prevent unnatural movement

3. **Continuous Joints**: Allow unlimited rotation around a single axis
   - In URDF: `<joint type="continuous">`
   - Example: A wrist joint that can rotate indefinitely (though often limited in practice)

4. **Spherical Joints**: Mimic ball-and-socket joints, allowing rotation in multiple axes
   - In URDF: These are typically approximated using multiple revolute joints with perpendicular axes
   - Example: Shoulder and hip joints

5. **Prismatic Joints**: Allow linear translation along a single axis
   - In URDF: `<joint type="prismatic">`
   - Less common in humanoid robots but could be used for telescoping mechanisms

#### Joint Constraints for Humanoid Robots

Humanoid robots must model realistic human joint constraints to function effectively:

**Shoulder Joints (Ball and Socket)**:
- Requires multiple DOF to approximate spherical movement
- Typically modeled with 3 revolute joints at 90-degree angles to each other
- Range of motion: ~±180° in flexion/extension, ~±90° in abduction/adduction

**Elbow Joints (Hinge)**:
- Single revolute joint in the Ulna/Radial axis
- Range of motion: ~-15° (slight hyperextension) to ~150° (full flexion)

**Wrist Joints**:
- Often modeled with 2 revolute joints (flexion/extension + radial/ulnar deviation)
- Range of motion: ~±80° flexion/extension, ~±45° deviation

**Hip Joints (Ball and Socket)**:
- Similar to shoulders, typically requires multiple DOF
- Range of motion: ~±120° flexion/extension, ~±45° abduction/adduction

**Knee Joints (Hinge)**:
- Single revolute joint primarily for flexion/extension
- Range of motion: ~0° (full extension) to ~135° (full flexion)

#### Joint Limits in URDF

Joint limits are critical in URDF for humanoid robots:

```xml
<joint name="elbow_joint" type="revolute">
  <parent link="upper_arm"/>
  <child link="lower_arm"/>
  <limit lower="0" upper="2.61799" effort="50" velocity="2"/>
  <!-- This limits the joint to 0-150 degrees (2.61799 radians) -->
</joint>
```

**Important considerations for joint limits**:
- Limits should reflect physical constraints of the actual hardware
- Safety margins should be built into the limits (don't use maximum theoretical values)
- Kinematic constraints of the human body should be respected
- Consider the interaction between multiple joints (e.g., shoulder and elbow movement)

### URDF Best Practices

When creating URDF files for humanoid robots, consider these best practices:

1. **Hierarchy**: Structure the robot as a tree with a single root link
2. **Units**: Use consistent units (typically meters for lengths, kilograms for masses)
3. **Joint Limits**: Specify appropriate limits for revolute joints to reflect physical constraints
4. **Inertial Properties**: Accurately model the mass and inertia for realistic physics simulation
5. **Collision vs Visual**: Use simpler shapes for collision than visual when possible to improve performance

## Hands-on Lab

### Lab Objective
Create a simple URDF file for a humanoid limb and visualize it using ROS 2 tools.

### Prerequisites
- ROS 2 Humble Hawksbill installed
- Basic understanding of XML
- Completed Chapter 1 and 2

### Steps
1. **Create a URDF file** for a simple humanoid arm (we'll create this in examples/humanoid-arm.urdf)

2. **Validate the URDF file** using the check_urdf tool

3. **Visualize the robot** using RViz or other visualization tools

4. **Examine the joint relationships** within the URDF

### Expected Output
- A valid URDF file describing a humanoid arm
- Successful visualization of the robot structure
- Understanding of how links and joints connect

### Validation Commands
```bash
# Check if the URDF is valid
check_urdf examples/simple-robot.urdf

# Parse and display URDF information
ros2 run urdf_parser urdf_to_graphiz examples/simple-robot.urdf

# Launch visualization (if available)
# rviz2 -d [config_file]
```

### Hints
- Start with simple geometric shapes for links
- Ensure all joints have properly defined parent and child links
- Use appropriate joint limits for humanoid joints
- Check the URDF specification when in doubt

## Comprehension Checks

### Question 1
What does URDF stand for?
A) Universal Robot Definition Format
B) Unified Robot Description Format
C) Universal Robot Design Framework
D) Unified Robotics Development Format

**Answer: B** - URDF stands for Unified Robot Description Format, which is an XML-based format for describing robots.

### Question 2
Which joint type would be most appropriate for a humanoid elbow joint?
A) Fixed
B) Continuous
C) Revolute
D) Prismatic

**Answer: C** - A revolute joint allows single-axis rotation, which is appropriate for an elbow joint that bends in one plane.

### Question 3
True or False: A URDF file can have multiple root links.
A) True
B) False

**Answer: B** - False. A URDF file must have exactly one root link, and all other links in the robot must be connected in a tree structure from this root.

## Summary

In this chapter, we've covered the fundamentals of URDF for humanoid robotics:
- URDF is an XML-based format for describing robot structure
- Links represent rigid parts of the robot
- Joints connect links and define allowed motion
- Humanoid robots have a specific hierarchical structure in URDF
- Best practices ensure valid and efficient robot descriptions

These concepts are essential for working with humanoid robots in ROS 2 environments.

## References and Citations

- ROS.org. (2023). *URDF: Unified Robot Description Format*. Available at: http://wiki.ros.org/urdf
- ROS.org. (2023). *URDF Tutorials*. Available at: http://wiki.ros.org/urdf/Tutorials
- Smart, W. (2023). *Robotics: URDF Documentation*. Available at: https://docs.ros.org/en/humble/p/urdf/
- Corke, P. (2022). *Springer Handbook of Robotics: Robot Modeling*. In B. Siciliano & O. Khatib (Eds.), Springer Handbook of Robotics (2nd ed.). Springer.