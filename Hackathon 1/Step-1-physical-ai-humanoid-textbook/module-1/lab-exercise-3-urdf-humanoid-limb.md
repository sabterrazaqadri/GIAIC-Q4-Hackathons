# Lab Exercise 3: Create URDF File for Humanoid Limb

## Objective

In this lab, you will create a URDF file for a humanoid limb (arm or leg) from scratch. You will learn how to structure a URDF file with appropriate links, joints, and visual/collision properties for a humanoid robot component.

## Prerequisites

- Completed Chapter 3 on URDF fundamentals
- Basic understanding of XML
- Understanding of humanoid robot anatomy and kinematics

## Estimated Duration

60 minutes

## Instructions

### Part A: Creating a Left Arm URDF

### Step 1: Define the root link
Start by defining the shoulder link as your root element. This will be connected to the torso in a full robot model.

### Step 2: Create the arm segments
Create appropriate links for:
- Upper arm
- Lower arm
- Hand

### Step 3: Connect with joints
Add joints connecting the segments with appropriate joint types:
- Shoulder (typically a revolute joint with multiple DOF, but we'll simplify to one axis)
- Elbow (revolute joint)
- Wrist (revolute joint)

### Step 4: Add visual and collision elements
For each link, add visual and collision elements with appropriate shapes and materials.

### Step 5: Add inertial properties
For each link, add mass and inertia properties that are realistic for a humanoid arm.

### Step 6: Validate your URDF
Use the validation tools to check that your URDF is properly formatted.

## Expected Output

A valid URDF file representing a humanoid left arm with:
- Appropriate link names (left_upper_arm, left_lower_arm, left_hand)
- Properly connected joints with realistic limits
- Visual elements with appropriate shapes and colors
- Collision elements
- Inertial properties for physics simulation

## Example Structure Template

```xml
<?xml version="1.0"?>
<robot name="left_arm">

  <!-- Link definitions here -->
  <!-- <link name="left_shoulder">...</link> -->
  <!-- <link name="left_upper_arm">...</link> -->
  <!-- <link name="left_lower_arm">...</link> -->
  <!-- <link name="left_hand">...</link> -->

  <!-- Joint definitions here -->
  <!-- <joint name="left_shoulder_joint" type="revolute">...</joint> -->
  <!-- <joint name="left_elbow_joint" type="revolute">...</joint> -->
  <!-- <joint name="left_wrist_joint" type="revolute">...</joint> -->

</robot>
```

## Validation Commands

```bash
# Validate your URDF file
check_urdf your_left_arm.urdf

# Generate a graph of the URDF structure
ros2 run urdf_parser urdf_to_graphiz your_left_arm.urdf

# Look at the generated .pdf to visualize the joint structure
```

## Troubleshooting Tips

- Ensure all joint parent/child relationships are properly defined
- Check that each link and joint has a unique name
- Verify that joint limits are realistic for human joints
- Make sure masses and inertias are reasonable values
- Use consistent units (meters for length, kilograms for mass)

## Extension Activities (Optional)

1. Add more DOF to the shoulder joint to make it more realistic
2. Create a leg URDF following the same principles
3. Add realistic joint limits based on human anatomy

## Assessment

Complete the following to verify your understanding:
1. Draw the kinematic chain of your arm from torso to hand
2. Explain why the choice of joint types is appropriate for the arm
3. Identify which parts of your URDF would affect simulation performance
4. Describe how you would modify your URDF to make it part of a complete humanoid