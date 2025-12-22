#!/usr/bin/env python3
"""
Verification Script for ROS 2 Code Examples

This script verifies that the code examples in the educational module
are properly structured for ROS 2 Humble and follow best practices.
"""

import ast
import sys
import os
from typing import List, Tuple, Dict

def check_imports(file_path: str) -> List[str]:
    """
    Check if the file has the correct ROS 2 imports.
    """
    errors = []
    with open(file_path, 'r', encoding='utf-8') as f:
        try:
            tree = ast.parse(f.read())
        except SyntaxError as e:
            errors.append(f"Syntax error in {file_path}: {e}")
            return errors
    
    has_rclpy = False
    has_std_msgs = False
    has_node_import = False
    
    for node in ast.walk(tree):
        if isinstance(node, ast.ImportFrom):
            if node.module and node.module.startswith('rclpy'):
                has_rclpy = True
            if node.module and 'std_msgs' in node.module:
                has_std_msgs = True
            if node.module and ('node' in node.module.lower() or 'Node' in str(node.names)):
                has_node_import = True
        elif isinstance(node, ast.Import):
            for alias in node.names:
                if alias.name.startswith('rclpy'):
                    has_rclpy = True
                if 'std_msgs' in alias.name:
                    has_std_msgs = True
    
    if not has_rclpy:
        errors.append(f"Missing rclpy import in {file_path}")
    if not has_node_import:
        errors.append(f"Missing Node import in {file_path}")
    
    return errors

def check_node_structure(file_path: str) -> List[str]:
    """
    Check if the file has proper ROS 2 node structure.
    """
    errors = []
    with open(file_path, 'r', encoding='utf-8') as f:
        content = f.read()
    
    # Check for basic node elements
    if 'rclpy.init' not in content:
        errors.append(f"Missing rclpy.init() call in {file_path}")
    
    if 'rclpy.shutdown' not in content:
        errors.append(f"Missing rclpy.shutdown() call in {file_path}")
    
    if 'create_publisher' not in content and 'create_subscription' not in content and 'create_service' not in content and 'create_client' not in content:
        errors.append(f"No ROS 2 entity creation found in {file_path} (expected publisher, subscription, service, or client)")
    
    # Check for proper node inheritance if there are classes
    if 'class' in content and 'Node' in content:
        lines = content.split('\n')
        for i, line in enumerate(lines):
            if 'class' in line and '(' in line and ')' in line:
                if 'Node' in line and 'rclpy.node' not in content and 'from rclpy.node import Node' not in content:
                    errors.append(f"Node class in {file_path} may not be importing Node correctly (line {i+1})")
    
    return errors

def verify_code_examples() -> Dict[str, List[str]]:
    """
    Verify all code examples in the examples directory.
    """
    example_files = [
        'examples/publisher-example.py',
        'examples/subscriber-example.py',
        'examples/service-server-example.py',
        'examples/service-client-example.py'
    ]
    
    results = {}
    
    for file_path in example_files:
        if not os.path.exists(file_path):
            results[file_path] = [f"File does not exist: {file_path}"]
            continue
        
        print(f"Verifying {file_path}...")
        
        import_errors = check_imports(file_path)
        structure_errors = check_node_structure(file_path)
        
        all_errors = import_errors + structure_errors
        results[file_path] = all_errors
        
        if all_errors:
            print(f"  Found {len(all_errors)} issues:")
            for error in all_errors:
                print(f"    - {error}")
        else:
            print(f"  [PASS] {file_path} passed verification")

    return results

def main():
    """
    Main function to run the verification.
    """
    print("Verifying ROS 2 code examples for compatibility with ROS 2 Humble...")
    print("=" * 60)
    
    results = verify_code_examples()
    
    print("\nSummary:")
    print("=" * 60)

    total_errors = 0
    for file_path, errors in results.items():
        if errors:
            print(f"[FAIL] {file_path}: {len(errors)} error(s)")
            total_errors += len(errors)
        else:
            print(f"[PASS] {file_path}: OK")

    print("=" * 60)
    if total_errors == 0:
        print("[PASS] All code examples passed verification for ROS 2 Humble compatibility!")
        print("\nNote: This script checks for proper structure and imports, but actual")
        print("    functionality would require a running ROS 2 Humble environment.")
        return 0
    else:
        print(f"[FAIL] Found {total_errors} total error(s) in code examples.")
        return 1

if __name__ == "__main__":
    sys.exit(main())