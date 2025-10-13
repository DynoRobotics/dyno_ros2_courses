#!/usr/bin/env python3
"""
Integration Script for ros2_zenoh_python

This script integrates generated simplified types into the ros2_zenoh_python.messages module.
"""

import os
import sys
import argparse
from pathlib import Path


def integrate_generated_types(package_name: str, generated_file: str, messages_module_path: str):
    """Integrate generated types into the messages module."""
    
    # Read the generated file
    with open(generated_file, 'r') as f:
        generated_content = f.read()
    
    # Read the existing messages module
    with open(messages_module_path, 'r') as f:
        existing_content = f.read()
    
    # Extract the generated classes (everything after imports)
    lines = generated_content.split('\n')
    class_start = 0
    for i, line in enumerate(lines):
        if line.startswith('@dataclass'):
            class_start = i
            break
    
    # Extract classes and __all__ list
    class_lines = lines[class_start:]
    
    # Find the __all__ list in generated content
    generated_all = None
    for line in class_lines:
        if line.startswith('__all__'):
            generated_all = line
            break
    
    # Extract class names from __all__
    if generated_all:
        # Parse __all__ = ['Class1', 'Class2', ...]
        all_content = generated_all.split('=')[1].strip()
        class_names = eval(all_content)  # Safe since we control the content
    else:
        # Fallback: extract class names from @dataclass lines
        class_names = []
        for line in class_lines:
            if line.startswith('class '):
                class_name = line.split()[1].split(':')[0]
                class_names.append(class_name)
    
    # Add the generated classes to the existing module
    integration_content = f"""
# Generated simplified types for {package_name} package
{chr(10).join(class_lines)}
"""
    
    # Update the existing content
    updated_content = existing_content + integration_content
    
    # Update __all__ list
    if '__all__' in existing_content:
        # Find existing __all__ list
        lines = updated_content.split('\n')
        all_start = -1
        all_end = -1
        
        for i, line in enumerate(lines):
            if line.strip().startswith('__all__'):
                all_start = i
                # Find the end of the list
                for j in range(i + 1, len(lines)):
                    if lines[j].strip() == ']':
                        all_end = j
                        break
                break
        
        if all_start != -1 and all_end != -1:
            # Extract existing items
            existing_all_line = lines[all_start]
            if '[' in existing_all_line and ']' in existing_all_line:
                # Single line __all__
                existing_items = eval(existing_all_line.split('=')[1].strip())
                all_items = existing_items + class_names
                lines[all_start] = f"__all__ = {all_items}"
            else:
                # Multi-line __all__
                existing_items = []
                for j in range(all_start + 1, all_end):
                    line = lines[j].strip()
                    if line.startswith("'") or line.startswith('"'):
                        item = line.strip("',\"")
                        existing_items.append(item)
                
                all_items = existing_items + class_names
                new_all_lines = [f"__all__ = {all_items}"]
                lines = lines[:all_start] + new_all_lines + lines[all_end + 1:]
        
        updated_content = '\n'.join(lines)
    else:
        # Add __all__ list if it doesn't exist
        updated_content += f"\n__all__ = {class_names}\n"
    
    # Write the updated content back
    with open(messages_module_path, 'w') as f:
        f.write(updated_content)
    
    print(f"✅ Integrated {len(class_names)} types from {package_name} into {messages_module_path}")
    print(f"📦 Added types: {', '.join(class_names)}")


def main():
    parser = argparse.ArgumentParser(description='Integrate generated types into ros2_zenoh_python.messages')
    parser.add_argument('--package', '-p', required=True,
                       help='Package name (e.g., geometry_msgs)')
    parser.add_argument('--generated-file', '-f', required=True,
                       help='Path to generated messages file')
    parser.add_argument('--messages-module', '-m', 
                       default='src/ros2_zenoh_python/ros2_zenoh_python/messages.py',
                       help='Path to messages.py module')
    parser.add_argument('--dry-run', action='store_true',
                       help='Show what would be integrated without making changes')
    
    args = parser.parse_args()
    
    # Validate files exist
    if not os.path.exists(args.generated_file):
        print(f"Error: Generated file '{args.generated_file}' not found")
        sys.exit(1)
    
    if not os.path.exists(args.messages_module):
        print(f"Error: Messages module '{args.messages_module}' not found")
        sys.exit(1)
    
    if args.dry_run:
        print(f"Would integrate {args.package} types from {args.generated_file} into {args.messages_module}")
        # Show what would be added
        with open(args.generated_file, 'r') as f:
            content = f.read()
            lines = content.split('\n')
            for line in lines:
                if line.startswith('class '):
                    print(f"  - {line.split()[1].split(':')[0]}")
    else:
        integrate_generated_types(args.package, args.generated_file, args.messages_module)


if __name__ == '__main__':
    main()
