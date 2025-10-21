"""
Setup script for ros2_zenoh_python package.
"""

from setuptools import setup, find_packages
import os

# Get the directory containing this setup.py file
here = os.path.abspath(os.path.dirname(__file__))

# Read the README file if it exists
readme_path = os.path.join(here, "README.md")
if os.path.exists(readme_path):
    with open(readme_path, encoding="utf-8") as f:
        long_description = f.read()
else:
    long_description = "ROS 2-compatible Python package using Zenoh as transport"

setup(
    name="ros2_zenoh_python",
    version="0.1.0",
    author="ROS 2 Zenoh Team",
    author_email="zenoh@zettascale.tech",
    description="ROS 2-compatible Python package using Zenoh as transport",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/eclipse-zenoh/ros2_zenoh_python",
    packages=find_packages(),
    classifiers=[
        "Development Status :: 3 - Alpha",
        "Intended Audience :: Developers",
        "License :: OSI Approved :: Apache Software License",
        "License :: OSI Approved :: Eclipse Public License 2.0 (EPL-2.0)",
        "Operating System :: OS Independent",
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.8",
        "Programming Language :: Python :: 3.9",
        "Programming Language :: Python :: 3.10",
        "Programming Language :: Python :: 3.11",
        "Programming Language :: Python :: 3.12",
        "Topic :: Software Development :: Libraries :: Python Modules",
        "Topic :: System :: Networking",
    ],
    python_requires=">=3.8",
    install_requires=[
        "eclipse-zenoh>=1.5.0",
        "pycdr2>=0.1.0",
    ],
    extras_require={
        "dev": [
            "pytest>=6.0",
            "pytest-cov>=2.0",
            "black>=21.0",
            "flake8>=3.8",
            "mypy>=0.800",
        ],
        "ros2": [
            "rclpy>=3.0",
            "geometry_msgs",
            "builtin_interfaces",
            "rcl_interfaces",
        ],
    },
    # entry_points={
    #     "console_scripts": [
    #         "ros2-zenoh-pub=examples.publisher_example:main",
    #         "ros2-zenoh-sub=examples.subscriber_example:main",
    #     ],
    # },
)
