#!/usr/bin/env python3
"""
URDF Validation Script for ROS 2 Fundamentals Module

This script provides functionality to validate URDF files used in the educational module.
It checks for basic syntax and structure requirements without requiring a full ROS installation.
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
    
    # Check file extension
    if not file_path.lower().endswith(('.urdf', '.xacro')):
        errors.append(f"File does not have .urdf or .xacro extension: {file_path}")
    
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


def validate_urdf_semantics(file_path: str) -> Tuple[bool, List[str]]:
    """
    Validates the semantic structure of a URDF file.
    
    Args:
        file_path (str): Path to the URDF file to validate
        
    Returns:
        Tuple[bool, List[str]]: (is_valid, list_of_errors)
    """
    errors = []
    
    try:
        tree = ET.parse(file_path)
        root = tree.getroot()
        
        if root.tag == 'robot':
            errors.extend(validate_robot_structure(root))
    
    except Exception as e:
        errors.append(f"Error during semantic validation: {str(e)}")
        return False, errors
    
    return len(errors) == 0, errors


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
    is_semantic_valid, semantic_errors = validate_urdf_semantics(file_path)
    
    all_errors = syntax_errors + semantic_errors
    return is_semantic_valid, all_errors


def main():
    """Main function to run the URDF validator from command line."""
    if len(sys.argv) != 2:
        print("Usage: python urdf_validator.py <path_to_urdf_file>")
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