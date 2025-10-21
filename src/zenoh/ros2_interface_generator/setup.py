#!/usr/bin/env python3
"""Setup script for ros2_interface_generator."""

from setuptools import setup, find_packages
from pathlib import Path

# Read README
readme_file = Path(__file__).parent / "README.md"
long_description = readme_file.read_text() if readme_file.exists() else ""

setup(
    name="ros2_interface_generator",
    version="0.1.0",
    description="Universal code generator for ROS2 interfaces (messages, services, actions) supporting multiple languages and encodings",
    long_description=long_description,
    long_description_content_type="text/markdown",
    author="ROS2 Zenoh Community",
    url="https://github.com/your-org/ros2_interface_generator",
    packages=find_packages(),
    scripts=['bin/ros2-generate-interfaces'],
    install_requires=[
        # No required dependencies - all backends are optional
    ],
    extras_require={
        'python': ['pycdr2'],  # For Python CDR backend
        'dev': ['pytest', 'black', 'mypy'],
    },
    python_requires=">=3.8",
    classifiers=[
        "Development Status :: 3 - Alpha",
        "Intended Audience :: Developers",
        "Topic :: Software Development :: Code Generators",
        "License :: OSI Approved :: Apache Software License",
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.8",
        "Programming Language :: Python :: 3.9",
        "Programming Language :: Python :: 3.10",
        "Programming Language :: Python :: 3.11",
        "Programming Language :: Python :: 3.12",
    ],
    keywords="ros2 interfaces code-generation cdr zenoh",
    project_urls={
        "Bug Reports": "https://github.com/your-org/ros2_interface_generator/issues",
        "Source": "https://github.com/your-org/ros2_interface_generator",
    },
)

