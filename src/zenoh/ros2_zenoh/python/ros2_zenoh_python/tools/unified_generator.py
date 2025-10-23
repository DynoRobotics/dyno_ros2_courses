#!/usr/bin/env python3
"""
Unified ROS 2 Message Generator and Integrator

This script generates simplified ROS 2 message types and integrates them directly
into the ros2_zenoh_python package in one step.
"""

import os
import sys
import argparse
from pathlib import Path

# Import our existing generator
from generate_simplified_types import find_ros2_msg_files, ROS2InterfaceParser, SimplifiedTypeGenerator


def integrate_into_package(package_name: str, generated_content: str, target_package_dir: str):
    """Integrate generated content directly into the ros2_zenoh_python package."""
    
    # Create package directory structure in target
    package_dir = os.path.join(target_package_dir, package_name)
    msg_dir = os.path.join(package_dir, 'msg')
    os.makedirs(msg_dir, exist_ok=True)
    
    # Write package __init__.py
    package_init = f'# {package_name} package\n'
    with open(os.path.join(package_dir, '__init__.py'), 'w') as f:
        f.write(package_init)
    
    # Write msg __init__.py
    msg_init = f'# {package_name}.msg package\n'
    with open(os.path.join(msg_dir, '__init__.py'), 'w') as f:
        f.write(msg_init)
    
    # Write the main message file
    output_file = os.path.join(msg_dir, f"{package_name}.py")
    with open(output_file, 'w') as f:
        f.write(generated_content)
    
    return output_file


def update_package_init(package_name: str, message_classes: list, target_package_dir: str):
    """Update the main package __init__.py to export the new message types."""
    
    main_init_file = os.path.join(target_package_dir, '__init__.py')
    
    # Read existing content
    if os.path.exists(main_init_file):
        with open(main_init_file, 'r') as f:
            existing_content = f.read()
    else:
        existing_content = f'# ros2_zenoh_python package\n'
    
    # Add import for the new package
    import_line = f"from .{package_name}.msg.{package_name} import {', '.join(message_classes)}\n"
    
    # Check if import already exists
    if import_line.strip() not in existing_content:
        # Add the import
        if existing_content.strip():
            existing_content += "\n" + import_line
        else:
            existing_content = import_line
        
        # Update __all__ list
        if '__all__' in existing_content:
            # Find and update existing __all__
            lines = existing_content.split('\n')
            for i, line in enumerate(lines):
                if line.strip().startswith('__all__'):
                    # Extract existing items
                    if '[' in line and ']' in line:
                        existing_items = eval(line.split('=')[1].strip())
                        all_items = existing_items + message_classes
                        lines[i] = f"__all__ = {all_items}"
                        break
            existing_content = '\n'.join(lines)
        else:
            # Add __all__ list
            existing_content += f"\n__all__ = {message_classes}\n"
        
        # Write updated content
        with open(main_init_file, 'w') as f:
            f.write(existing_content)
        
        print(f"✅ Updated {main_init_file} to export {package_name} types")


def main():
    parser = argparse.ArgumentParser(description='Unified ROS 2 message generator and integrator')
    parser.add_argument('--package', '-p',
                       help='Specific package to process (if not specified, processes all)')
    parser.add_argument('--target-package', '-t', 
                       default='src/ros2_interfaces_python/ros2_interfaces_python',
                       help='Target package directory for integration')
    parser.add_argument('--verbose', '-v', action='store_true',
                       help='Enable verbose output')
    parser.add_argument('--dry-run', action='store_true',
                       help='Show what would be generated without creating files')
    
    args = parser.parse_args()
    
    # Auto-detect ROS 2 message files
    print("Auto-detecting ROS 2 message files...")
    msg_files = find_ros2_msg_files()
    if not msg_files:
        print("Error: No ROS 2 message files found. Please ensure ROS 2 is properly installed.")
        sys.exit(1)
    print(f"Found {len(msg_files)} message files from auto-detection")
    
    # Create target directory
    os.makedirs(args.target_package, exist_ok=True)
    
    parser_obj = ROS2InterfaceParser()
    generator = SimplifiedTypeGenerator()
    
    # Group messages by package
    packages = {}
    for msg_file in msg_files:
        path_parts = Path(msg_file).parts
        package_name = None
        for i, part in enumerate(path_parts):
            if part == 'msg' and i > 0:
                package_name = path_parts[i-1]
                break
        
        if package_name:
            if package_name not in packages:
                packages[package_name] = []
            packages[package_name].append(msg_file)
    
    # Process each package
    integrated_packages = []
    for package_name, package_msg_files in packages.items():
        if args.package and package_name != args.package:
            continue
            
        print(f"\nProcessing package: {package_name}")
        
        messages = []
        for msg_file in package_msg_files:
            if args.verbose:
                print(f"  Parsing: {msg_file}")
            
            try:
                message_info = parser_obj.parse_msg_file(msg_file)
                messages.append(message_info)
                print(f"  ✓ {message_info.name} ({len(message_info.fields)} fields)")
            except Exception as e:
                print(f"  ✗ Error parsing {msg_file}: {e}")
        
        if messages:
            # Generate Python module
            module_content = generator.generate_package_module(messages, package_name)
            
            if args.dry_run:
                print(f"  Would integrate {len(messages)} messages into {args.target_package}/{package_name}/")
                class_names = [msg.name for msg in messages]
                print(f"  Classes: {', '.join(class_names)}")
            else:
                # Integrate directly into target package
                output_file = integrate_into_package(package_name, module_content, args.target_package)
                
                # Update package __init__.py
                class_names = [msg.name for msg in messages]
                update_package_init(package_name, class_names, args.target_package)
                
                integrated_packages.append(package_name)
                print(f"  ✅ Integrated into: {output_file}")
    
    if not args.dry_run and integrated_packages:
        print(f"\n🎉 Successfully integrated {len(integrated_packages)} packages:")
        for package in integrated_packages:
            print(f"  ✅ {package}")
        
        print(f"\n📝 Integration complete!")
        print(f"   Messages are now available as: from ros2_zenoh_python.{package_name}.msg.{package_name} import MessageType")
        print(f"   Or: from ros2_zenoh_python import MessageType")


if __name__ == '__main__':
    main()
