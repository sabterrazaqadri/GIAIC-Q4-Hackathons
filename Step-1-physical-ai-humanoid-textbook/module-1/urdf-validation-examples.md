# URDF Validation Examples and Instructions

## Purpose
This document provides examples and instructions for validating URDF files, ensuring they are properly structured and will work correctly in ROS 2 environments.

## Validation Tools

### 1. check_urdf
The primary tool for validating URDF files is `check_urdf`, which will parse your URDF and report any errors or warnings.

#### Usage:
```bash
check_urdf [path_to_urdf_file]
```

#### Example:
```bash
# Validate the simple robot URDF
check_urdf examples/simple-robot.urdf

# Validate the humanoid limb URDF
check_urdf examples/humanoid-limb.urdf
```

#### Expected Output for Valid URDF:
```
robot name is: simple_robot
---------- Successfully Parsed XML ---------------
root Link: base_link has 1 child links
child(1):  arm
child(1,0):  gripper
```

### 2. urdf_to_graphiz
This tool generates a graphical representation of the URDF's kinematic structure.

#### Usage:
```bash
ros2 run urdf_parser urdf_to_graphiz [path_to_urdf_file]
```

#### Example:
```bash
# Generate a graph of the simple robot structure
ros2 run urdf_parser urdf_to_graphiz examples/simple-robot.urdf
# This will create simple_robot.gv and simple_robot.pdf files
```

## Validation Examples

### Example 1: Validating the Simple Robot URDF

```bash
$ check_urdf examples/simple-robot.urdf
robot name is: simple_robot
---------- Successfully Parsed XML ---------------
root Link: base_link has 1 child links
child(1):  arm
child(1,0):  gripper
```

### Example 2: Validating the Humanoid Limb URDF

```bash
$ check_urdf examples/humanoid-limb.urdf
robot name is: humanoid_limb
---------- Successfully Parsed XML ---------------
root Link: torso has 1 child links
child(1):  right_upper_arm
child(1,0):  right_lower_arm
child(1,0,0):  right_hand
```

## Common Validation Issues and Fixes

### Issue 1: Missing Parent/Child Relationships
**Error**: "No parent joint found for link"
**Fix**: Ensure every link (except the root) has a joint that connects it to another link

### Issue 2: Invalid Joint Types
**Error**: "Unknown joint type"
**Fix**: Use only valid joint types: fixed, revolute, continuous, prismatic, floating, planar

### Issue 3: Missing Required Elements
**Error**: "Missing required element [element_name]"
**Fix**: Ensure all required elements are present (e.g., parent, child for joints)

### Issue 4: XML Syntax Errors
**Error**: "XML parsing error"
**Fix**: Check for proper XML syntax, matching tags, and proper escaping of special characters

## Validation Script Example

Here's a simple Python script to validate URDF files:

```python
#!/usr/bin/env python3
"""
URDF Validation Script
This script provides programmatic validation of URDF files.
"""

import xml.etree.ElementTree as ET
import sys
import os
from typing import List, Tuple


def validate_urdf_syntax(file_path: str) -> Tuple[bool, List[str]]:
    """
    Validates the basic XML syntax of a URDF file.
    
    Args:
        file_path (str): Path to the URDF file to validate
        
    Returns:
        Tuple[bool, List[str]]: (is_valid, list_of_errors)
    """
    errors = []
    
    # Check if file exists
    if not os.path.exists(file_path):
        errors.append(f"File does not exist: {file_path}")
        return False, errors
    
    try:
        # Parse the XML
        tree = ET.parse(file_path)
        root = tree.getroot()
        
        # Check root tag
        if root.tag != 'robot':
            errors.append(f"Root element is '{root.tag}', expected 'robot'")
        else:
            # Validate robot element has required 'name' attribute
            if 'name' not in root.attrib:
                errors.append("Root 'robot' element missing required 'name' attribute")
    
    except ET.ParseError as e:
        errors.append(f"XML parsing error: {str(e)}")
        return False, errors
    except Exception as e:
        errors.append(f"Error reading file: {str(e)}")
        return False, errors
    
    return len(errors) == 0, errors


def validate_robot_structure(root: ET.Element) -> List[str]:
    """
    Validates the basic structure of a robot definition in URDF.
    
    Args:
        root (ET.Element): Root element of the parsed URDF
        
    Returns:
        List[str]: List of structural validation errors
    """
    errors = []
    
    # Check for at least one link
    links = root.findall('link')
    if len(links) == 0:
        errors.append("Robot must have at least one 'link' element")
    
    # Check for proper naming of links
    link_names = set()
    for link in links:
        if 'name' not in link.attrib:
            errors.append(f"Link element missing required 'name' attribute")
        else:
            link_name = link.attrib['name']
            if link_name in link_names:
                errors.append(f"Duplicate link name found: {link_name}")
            else:
                link_names.add(link_name)
    
    # Check for at least one joint if there are multiple links
    joints = root.findall('joint')
    if len(links) > 1 and len(joints) == 0:
        errors.append("Multiple links present but no joints to connect them")
    
    # Validate joint structure
    joint_names = set()
    for joint in joints:
        if 'name' not in joint.attrib:
            errors.append(f"Joint element missing required 'name' attribute")
        else:
            joint_name = joint.attrib['name']
            if joint_name in joint_names:
                errors.append(f"Duplicate joint name found: {joint_name}")
            else:
                joint_names.add(joint_name)
        
        if 'type' not in joint.attrib:
            errors.append(f"Joint '{joint.attrib.get('name', 'unnamed')}' missing required 'type' attribute")
        else:
            valid_types = ['revolute', 'continuous', 'prismatic', 'fixed', 'floating', 'planar']
            joint_type = joint.attrib['type']
            if joint_type not in valid_types:
                errors.append(f"Joint '{joint.attrib.get('name', 'unnamed')}' has invalid type '{joint_type}', must be one of {valid_types}")
        
        # Check for required child elements in joint
        required_children = ['parent', 'child']
        for child_tag in required_children:
            child_element = joint.find(child_tag)
            if child_element is None:
                errors.append(f"Joint '{joint.attrib.get('name', 'unnamed')}' missing required '{child_tag}' element")
            elif 'link' not in child_element.attrib:
                errors.append(f"Joint '{joint.attrib.get('name', 'unnamed')}' {child_tag} element missing required 'link' attribute")
    
    return errors


def validate_urdf_complete(file_path: str) -> Tuple[bool, List[str]]:
    """
    Performs complete validation of a URDF file (syntax + semantics).
    
    Args:
        file_path (str): Path to the URDF file to validate
        
    Returns:
        Tuple[bool, List[str]]: (is_valid, list_of_errors)
    """
    # First validate syntax
    is_syntax_valid, syntax_errors = validate_urdf_syntax(file_path)
    if not is_syntax_valid:
        return False, syntax_errors
    
    # Then validate semantics
    try:
        tree = ET.parse(file_path)
        root = tree.getroot()
        
        if root.tag == 'robot':
            semantic_errors = validate_robot_structure(root)
        else:
            semantic_errors = [f"Root element is '{root.tag}', expected 'robot'"]
    
    except Exception as e:
        return False, [f"Error during semantic validation: {str(e)}"]
    
    all_errors = syntax_errors + semantic_errors
    return len(all_errors) == 0, all_errors


def main():
    """Main function to run the URDF validator from command line."""
    if len(sys.argv) != 2:
        print("Usage: python urdf_validation_examples.py <path_to_urdf_file>")
        sys.exit(1)
    
    file_path = sys.argv[1]
    
    print(f"Validating URDF file: {file_path}")
    print("-" * 50)
    
    is_valid, errors = validate_urdf_complete(file_path)
    
    if is_valid:
        print("✓ URDF file is valid!")
    else:
        print("✗ URDF validation failed with the following errors:")
        for i, error in enumerate(errors, 1):
            print(f"  {i}. {error}")
    
    print(f"\nValidation complete. {'Success' if is_valid else 'Failed'}.")


if __name__ == "__main__":
    main()
```

## Best Practices for URDF Validation

1. Validate frequently: Check your URDF after each significant change
2. Use visualization tools: Visualize your robot to catch structural issues
3. Test in simulation: Load your URDF in a simulator to check for runtime issues
4. Follow naming conventions: Use consistent and descriptive names
5. Use realistic values: Ensure masses, inertias, and joint limits are physically plausible

## Troubleshooting Common Errors

- If getting an error about missing packages, make sure the `urdfdom` and `joint_state_publisher` packages are installed
- For visualization issues, ensure all required dependencies for RViz are installed
- For physics simulation problems, double-check inertial properties and joint limits

By following these validation examples and instructions, you can ensure your URDF files are properly structured and ready for use in ROS 2 environments.