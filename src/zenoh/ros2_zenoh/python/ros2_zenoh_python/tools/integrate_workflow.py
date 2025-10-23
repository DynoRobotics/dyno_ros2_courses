#!/usr/bin/env python3
"""
Complete Integration Workflow for ros2_zenoh_python

This script provides a complete workflow for generating and integrating
simplified types from ROS 2 packages into the ros2_zenoh_python module.
"""

import os
import sys
import argparse
import subprocess
from pathlib import Path


def run_command(cmd, description):
    """Run a command and handle errors."""
    print(f"🔄 {description}...")
    try:
        result = subprocess.run(cmd, shell=True, check=True, capture_output=True, text=True)
        print(f"✅ {description} completed")
        return result.stdout
    except subprocess.CalledProcessError as e:
        print(f"❌ {description} failed: {e}")
        if e.stderr:
            print(f"Error: {e.stderr}")
        sys.exit(1)


def generate_and_integrate_packages(package_list_file: str, output_dir: str, messages_module: str, dry_run: bool = False):
    """Generate and integrate types for multiple packages."""
    
    # Read package list
    with open(package_list_file, 'r') as f:
        packages = [line.strip() for line in f if line.strip() and not line.startswith('#')]
    
    print(f"📦 Processing {len(packages)} packages: {', '.join(packages)}")
    
    # Create output directory
    os.makedirs(output_dir, exist_ok=True)
    
    integrated_packages = []
    
    for package in packages:
        print(f"\n🔧 Processing package: {package}")
        
        # Generate integration code
        generated_file = os.path.join(output_dir, f"{package}_messages.py")
        
        cmd = f"python3 src/ros2_zenoh_python/tools/simple_type_generator.py --package {package} --integrate --output-dir {output_dir}"
        run_command(cmd, f"Generate integration code for {package}")
        
        if dry_run:
            print(f"  Would integrate {package} types")
            # Show what would be integrated
            cmd = f"python3 src/ros2_zenoh_python/tools/integrate_types.py --package {package} --generated-file {generated_file} --messages-module {messages_module} --dry-run"
            run_command(cmd, f"Show integration preview for {package}")
        else:
            # Integrate into messages module
            cmd = f"python3 src/ros2_zenoh_python/tools/integrate_types.py --package {package} --generated-file {generated_file} --messages-module {messages_module}"
            run_command(cmd, f"Integrate {package} types into messages module")
            integrated_packages.append(package)
    
    if not dry_run and integrated_packages:
        print(f"\n🎉 Successfully integrated {len(integrated_packages)} packages:")
        for package in integrated_packages:
            print(f"  ✅ {package}")
        
        print(f"\n📝 Next steps:")
        print(f"1. Update ros2_zenoh_python/__init__.py to export the new types")
        print(f"2. Test the integrated types with your applications")
        print(f"3. Update documentation if needed")


def create_package_list_file(packages: list, filename: str):
    """Create a package list file."""
    with open(filename, 'w') as f:
        f.write("# ROS 2 packages for simplified type generation\n")
        f.write("# Add or remove packages as needed\n\n")
        for package in packages:
            f.write(f"{package}\n")
    
    print(f"📝 Created package list file: {filename}")


def main():
    parser = argparse.ArgumentParser(description='Complete workflow for generating and integrating ROS 2 simplified types')
    parser.add_argument('--packages', '-p', nargs='+',
                       help='List of ROS 2 packages to process')
    parser.add_argument('--package-list', '-l',
                       help='File containing list of package names')
    parser.add_argument('--output-dir', '-d', default='generated_types',
                       help='Output directory for generated files')
    parser.add_argument('--messages-module', '-m', 
                       default='src/ros2_zenoh_python/ros2_zenoh_python/messages.py',
                       help='Path to messages.py module')
    parser.add_argument('--create-list', action='store_true',
                       help='Create a package list file from --packages')
    parser.add_argument('--dry-run', action='store_true',
                       help='Show what would be done without making changes')
    parser.add_argument('--list-file', default='package_list.txt',
                       help='Name of package list file to create')
    
    args = parser.parse_args()
    
    if args.create_list and args.packages:
        create_package_list_file(args.packages, args.list_file)
        return
    
    if args.package_list:
        if not os.path.exists(args.package_list):
            print(f"Error: Package list file '{args.package_list}' not found")
            sys.exit(1)
        
        generate_and_integrate_packages(args.package_list, args.output_dir, args.messages_module, args.dry_run)
    
    elif args.packages:
        # Create temporary package list file
        temp_list_file = 'temp_package_list.txt'
        create_package_list_file(args.packages, temp_list_file)
        
        try:
            generate_and_integrate_packages(temp_list_file, args.output_dir, args.messages_module, args.dry_run)
        finally:
            # Clean up temporary file
            if os.path.exists(temp_list_file):
                os.remove(temp_list_file)
    
    else:
        print("Error: Must specify either --packages or --package-list")
        parser.print_help()
        sys.exit(1)


if __name__ == '__main__':
    main()
